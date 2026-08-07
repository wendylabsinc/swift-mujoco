import CMuJoCo
import Foundation
import MuJoCo
import WendyMuJoCo
import RobotKit

/// MuJoCo-backed Unitree Go2 quadruped. Loads the real `unitree_go2` model
/// via `WendyMuJoCo.Menagerie` (fetching `mujoco_menagerie` on first use if
/// it isn't already vendored/cached) and drives it with a joint-space PD
/// controller, matching `go2.robot.json`'s numeric conventions exactly (see
/// the `static let`s below).
///
/// Not `Sendable` (it owns `MjModel`/`MjData`, neither of which is) — each
/// parallel rollout worker constructs its own instance, same as `CartpoleEnv`.
public final class Go2Environment: Environment {
    /// Declared joint order from `go2.robot.json`. NOTE: this does **not**
    /// match the real `unitree_go2` MJCF's actuator declaration order, which
    /// is FL, FR, RL, RR (not FR, FL, RR, RL as here) — see `actuatorIds`,
    /// which is resolved by name specifically to avoid a silent leg-swap bug
    /// from assuming the two orders line up positionally.
    static let jointOrder = [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    ]
    static let kp = [Double](repeating: 20, count: 12)
    static let kd = [Double](repeating: 0.5, count: 12)
    static let torqueLimits = [Double](repeating: 23.5, count: 12)
    static let defaultPos: [Double] = Array(repeating: [0.0, 0.8, -1.5], count: 4).flatMap { $0 }
    static let actionScale: [Double] = Array(repeating: [0.125, 0.25, 0.25], count: 4).flatMap { $0 }
    static let fallHeightThreshold = 0.18
    static let fallUprightThreshold = 0.5
    /// `go2.robot.json`'s `control.spawn_height`. The MJCF's own compiled
    /// default pose (`qpos0`, what a bare `mj_resetData` restores) has every
    /// leg joint at angle 0 — which is outside the knee joint's declared
    /// range (`-2.7227` to `-0.83776` for every `*_calf_joint`) and produces
    /// an immediately-unstable pose. `reset()` instead places the robot at
    /// `defaultPos` (the PD controller's own nominal target) at this height,
    /// upright, with zero velocity — a physically valid standing pose.
    static let spawnHeight = 0.32

    private let model: MjModel
    private let data: MjData
    private let velocityCommand: (vx: Double, vy: Double, wz: Double)
    private var previousAction = [Double](repeating: 0, count: 12)

    /// (qpos address, qvel/dof address) for each of `jointOrder`, resolved
    /// once at init via `MjModel.id(of:name:)` — every Go2 leg joint is a
    /// 1-DOF hinge, so each has exactly one qpos slot and one dof slot.
    private let jointAddresses: [(qpos: Int, dof: Int)]
    /// Actuator id for each of `jointOrder`, resolved by name (actuator
    /// names mirror joint names with the `_joint` suffix dropped, e.g.
    /// `FR_hip_joint` -> actuator `FR_hip`). Resolving by name — rather than
    /// assuming actuator id `i` drives `jointOrder[i]` — is what avoids the
    /// FL/FR ordering mismatch described on `jointOrder`.
    private let actuatorIds: [Int]
    /// qpos/dof address of the base body's free joint (7 qpos: xyz + wxyz
    /// quat; 6 dof: linear vel + local-frame angular vel).
    private let baseQposAdr: Int
    private let baseDofAdr: Int

    // Menagerie.fetch (invoked by Menagerie.load below on a cache miss) shells
    // out to `git clone`/`git sparse-checkout add` against one shared cache
    // directory with no locking of its own — two Go2Environments constructed
    // concurrently (e.g. parallel rollout workers) race on git's own lock
    // files and one throws. Serializing the load process-wide avoids that;
    // it only costs anything on the first, cache-populating call.
    private static let modelLoadLock = NSLock()

    public init(velocityCommand: (vx: Double, vy: Double, wz: Double) = (0, 0, 0)) {
        // Menagerie.load resolves "go2" -> "unitree_go2", preferring a vendored
        // copy under Menagerie.vendorDirs and falling back to a sparse git
        // clone of google-deepmind/mujoco_menagerie into WorldSim's cache dir.
        // With no vendored copy present, it returns unitree_go2/scene.xml
        // (floor + lights, robot included) rather than the bare robot XML.
        Self.modelLoadLock.lock()
        let model = try! Menagerie.load("go2")
        Self.modelLoadLock.unlock()
        self.model = model
        self.data = MjData(model)
        self.velocityCommand = velocityCommand

        self.jointAddresses = Self.jointOrder.map { name in
            guard let id = model.id(of: objJoint, name: name) else {
                preconditionFailure("Go2 model is missing expected joint \"\(name)\"")
            }
            let info = model.joints[id]
            return (qpos: info.qposadr, dof: info.dofadr)
        }
        self.actuatorIds = Self.jointOrder.map { jointName in
            let actuatorName = String(jointName.dropLast("_joint".count))
            guard let id = model.id(of: objActuator, name: actuatorName) else {
                preconditionFailure("Go2 model is missing expected actuator \"\(actuatorName)\"")
            }
            return id
        }
        guard let freeJoint = model.joints.first(where: { Int32($0.type) == Int32(mjJNT_FREE.rawValue) })
        else {
            preconditionFailure("Go2 model has no free joint for its base body")
        }
        self.baseQposAdr = freeJoint.qposadr
        self.baseDofAdr = freeJoint.dofadr
    }

    /// Exposes the underlying MuJoCo simulation for callers that need to feed
    /// a running rollout into `WendyMuJoCo.WorldSimRecorder` (e.g.
    /// go2-locomotion-demo's inference loop). Read-only — nothing outside
    /// this type should ever mutate `model`/`data` directly.
    public var mjModel: MjModel { model }
    public var mjData: MjData { data }

    public var isTerminated: Bool {
        let obs = observation()
        return obs.baseHeight < Self.fallHeightThreshold || obs.upright < Self.fallUprightThreshold
    }

    public func reset() -> Go2Observation {
        mjResetData(model, data)
        // Place the base upright at spawnHeight with zero velocity, and every
        // leg joint at its PD-target default pose (see `spawnHeight`'s doc).
        data.setQpos(at: baseQposAdr + 0, 0)
        data.setQpos(at: baseQposAdr + 1, 0)
        data.setQpos(at: baseQposAdr + 2, Self.spawnHeight)
        data.setQpos(at: baseQposAdr + 3, 1)  // quat w
        data.setQpos(at: baseQposAdr + 4, 0)  // quat x
        data.setQpos(at: baseQposAdr + 5, 0)  // quat y
        data.setQpos(at: baseQposAdr + 6, 0)  // quat z
        for (i, addr) in jointAddresses.enumerated() {
            data.setQpos(at: addr.qpos, Self.defaultPos[i])
        }
        mjForward(model, data)
        previousAction = [Double](repeating: 0, count: 12)
        return observation()
    }

    public func act(_ action: Go2Command) -> Go2Observation {
        for i in 0..<12 {
            let residual = action.jointPositionResiduals[i] * Self.actionScale[i]
            let target = Self.defaultPos[i] + residual
            let currentPos = data.qpos(at: jointAddresses[i].qpos)
            let currentVel = data.qvel(at: jointAddresses[i].dof)
            let torque = Self.kp[i] * (target - currentPos) - Self.kd[i] * currentVel
            let clipped = max(-Self.torqueLimits[i], min(Self.torqueLimits[i], torque))
            data.setCtrl(actuatorIds[i], clipped)
        }
        mjStep(model, data)
        previousAction = action.jointPositionResiduals
        return observation()
    }

    private func observation() -> Go2Observation {
        let baseQuat = Quat(
            w: data.qpos(at: baseQposAdr + 3),
            x: data.qpos(at: baseQposAdr + 4),
            y: data.qpos(at: baseQposAdr + 5),
            z: data.qpos(at: baseQposAdr + 6)
        )
        let baseHeight = data.qpos(at: baseQposAdr + 2)

        // Local "up" axis (0,0,1) rotated into world by the base's
        // orientation, dotted with world-up — 1.0 when perfectly upright,
        // falling toward 0 (and below) as the robot tips over.
        let localUp = rotVecQuat(Vec3(0, 0, 1), baseQuat)
        let upright = localUp.dot(Vec3(0, 0, 1))

        // World gravity direction rotated into the base's local frame (Rᵀ·v),
        // i.e. what an IMU accelerometer/orientation filter would report as
        // "which way is down" in body coordinates.
        let gravityDirection = Vec3(model.gravity.0, model.gravity.1, model.gravity.2).normalized
        let projectedGravity = quat2Mat(baseQuat).transposeTimes(gravityDirection)

        // No IMU gyro sensor is present on the vendored unitree_go2 model
        // (go2.robot.json's imu.gyro_sensor is added by wendy-sandbox's own
        // wrapper, not the raw Menagerie MJCF) — fall back to the base free
        // joint's own angular velocity. MuJoCo's free-joint qvel convention:
        // the linear part (dof 0..<3) is world-frame, but the angular part
        // (dof 3..<6) is already expressed in the body's local frame, which
        // is exactly what a body-mounted gyro would read.
        let angularVelocity = (
            data.qvel(at: baseDofAdr + 3),
            data.qvel(at: baseDofAdr + 4),
            data.qvel(at: baseDofAdr + 5)
        )

        // The free joint's linear qvel (dof 0..<3) is world-frame (unlike the
        // angular part above, which MuJoCo already expresses locally) — see
        // this file's other qvel comment. Rotate it into the base's local
        // frame with the same Rᵀ·v transform used for `projectedGravity`
        // above, so it's directly comparable to `velocityCommand`'s
        // body-frame vx/vy/wz.
        let worldLinearVelocity = Vec3(
            data.qvel(at: baseDofAdr + 0),
            data.qvel(at: baseDofAdr + 1),
            data.qvel(at: baseDofAdr + 2)
        )
        let localLinearVelocity = quat2Mat(baseQuat).transposeTimes(worldLinearVelocity)

        var jointPositions = [Double](repeating: 0, count: 12)
        var jointVelocities = [Double](repeating: 0, count: 12)
        for i in 0..<12 {
            jointPositions[i] = data.qpos(at: jointAddresses[i].qpos) - Self.defaultPos[i]
            jointVelocities[i] = data.qvel(at: jointAddresses[i].dof)
        }

        return Go2Observation(
            baseAngularVelocity: angularVelocity,
            baseLinearVelocity: (localLinearVelocity.x, localLinearVelocity.y, localLinearVelocity.z),
            projectedGravity: (projectedGravity.x, projectedGravity.y, projectedGravity.z),
            velocityCommand: velocityCommand,
            jointPositions: jointPositions,
            jointVelocities: jointVelocities,
            previousAction: previousAction,
            baseHeight: baseHeight,
            upright: upright
        )
    }
}

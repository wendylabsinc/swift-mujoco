// Sources/RobotKitSim/Go2Simulator.swift
import MuJoCo
import RobotKit
import RobotKitGo2

/// A MuJoCo Go2 that speaks the robot's own wire messages.
///
/// It consumes `unitree_go/LowCmd` and produces `unitree_go/LowState`, so the
/// adapter and decode path that will face hardware are exercised here too. The
/// PD law lives in this class because the Menagerie model's actuators are pure
/// torque motors with no built-in gains, exactly matching the real robot where
/// kp/kd arrive per command.
///
/// Not `Sendable`: it owns `MjModel`/`MjData`, neither of which is. Construct
/// one per simulation and keep it on a single task.
public final class Go2Simulator {
    /// Physics runs at the model timestep (2 ms for the Go2 model); control
    /// runs every 10th step, i.e. 50 Hz, matching a realistic policy rate.
    public static let controlDecimation = 10

    private let model: MjModel
    private let data: MjData
    /// `actuatorForJoint[c]` is the MuJoCo actuator index for canonical joint `c`.
    private let actuatorForJoint: [Int]
    /// `qposAddressForJoint[c]` / `dofAddressForJoint[c]`, canonical order.
    private let qposAddressForJoint: [Int]
    private let dofAddressForJoint: [Int]
    private let footGeomIds: [Int]
    private let gyroSensor: MjModel.SensorInfo?
    private let accelSensor: MjModel.SensorInfo?
    private let quatSensor: MjModel.SensorInfo?
    private var lastTorques: [Double]

    public convenience init(modelXMLPath: String) throws {
        try self.init(model: MjModel.load(xmlPath: modelXMLPath))
    }

    public convenience init(modelXML: String) throws {
        try self.init(model: MjModel.load(xml: modelXML))
    }

    private init(model: MjModel) {
        self.model = model
        self.data = MjData(model)

        let joints = model.joints
        let actuators = model.actuators
        // Canonical order is the firmware's; look each joint up by name so the
        // model's own declaration order never leaks into the mapping.
        var actuatorIndices: [Int] = []
        var qposAddresses: [Int] = []
        var dofAddresses: [Int] = []
        for name in Go2JointMap.unitreeOrder.names {
            guard let joint = joints.first(where: { $0.name == name }) else {
                preconditionFailure("model is missing joint '\(name)'")
            }
            qposAddresses.append(joint.qposadr)
            dofAddresses.append(joint.dofadr)
            // Actuator names drop the "_joint" suffix in both the Menagerie
            // model and the test fixture (FR_hip_joint -> FR_hip).
            let actuatorName = String(name.dropLast("_joint".count))
            guard let actuator = actuators.first(where: { $0.name == actuatorName }) else {
                preconditionFailure("model is missing actuator '\(actuatorName)'")
            }
            actuatorIndices.append(actuator.id)
        }
        self.actuatorForJoint = actuatorIndices
        self.qposAddressForJoint = qposAddresses
        self.dofAddressForJoint = dofAddresses
        self.footGeomIds = Go2JointMap.footGeomNames.compactMap { model.id(of: objGeom, name: $0) }
        self.gyroSensor = model.sensor(named: "rk_gyro")
        self.accelSensor = model.sensor(named: "rk_accel")
        self.quatSensor = model.sensor(named: "rk_quat")
        self.lastTorques = [Double](repeating: 0, count: Go2Adapter.jointCount)
    }

    public var time: Double { data.time }

    public func reset() {
        mjResetData(model, data)
        lastTorques = [Double](repeating: 0, count: Go2Adapter.jointCount)
        mjForward(model, data)
    }

    /// True when `cmd.crc` matches the checksum firmware would compute for it.
    ///
    /// Exposed as a boolean predicate — rather than inlined straight into
    /// `applyLowCmd`'s `preconditionFailure` — so a deliberately corrupted CRC
    /// can be exercised directly in a test without needing to trap a
    /// `preconditionFailure`, which the `Testing` framework cannot catch.
    static func hasValidCRC(_ cmd: LowCmd) -> Bool {
        cmd.crc == UnitreeCRC.crc(for: cmd)
    }

    /// Evaluates the PD law for every commanded joint and stores the resulting
    /// torques in `ctrl`. Called once per control tick; the torque then holds
    /// across the intervening physics steps, as it does on real hardware.
    ///
    /// A real Go2 silently drops a `LowCmd` whose CRC doesn't match. This
    /// simulator stands in for firmware, so it fails loudly instead: a CRC
    /// mismatch here means `packedLowCmdBytes`' offsets have drifted from the
    /// firmware's memory layout, exactly the class of bug this parity
    /// framework exists to catch, and a silent drop would let a loopback run
    /// print success over a broken wire format.
    public func applyLowCmd(_ cmd: LowCmd) {
        guard Self.hasValidCRC(cmd) else {
            preconditionFailure(
                "LowCmd CRC mismatch: got \(cmd.crc), expected \(UnitreeCRC.crc(for: cmd)) — "
                    + "command corrupted in transit or packedLowCmdBytes offsets drifted")
        }
        for c in 0..<Go2Adapter.jointCount {
            let motor = cmd.motorCmd[c]
            let q = data.qpos(at: qposAddressForJoint[c])
            let dq = data.qvel(at: dofAddressForJoint[c])
            let tau =
                Double(motor.kp) * (Double(motor.q) - q)
                + Double(motor.kd) * (Double(motor.dq) - dq)
                + Double(motor.tau)
            lastTorques[c] = tau
            data.setCtrl(actuatorForJoint[c], tau)
        }
    }

    public func step() {
        mjStep(model, data)
    }

    /// Synthesizes the robot's own state message from `mjData`.
    ///
    /// `state.crc` is left at its default (0): `UnitreeCRC` only defines the
    /// `LowCmd` checksum this plan's scope covers (the direction real
    /// firmware validates), and no `LowState`-flavored CRC function exists
    /// anywhere in this codebase to call — inventing one here would be an
    /// unreviewed checksum scheme, not a documented Unitree contract.
    public func lowState() -> LowState {
        var state = LowState()
        for c in 0..<Go2Adapter.jointCount {
            state.motorState[c].q = Float(data.qpos(at: qposAddressForJoint[c]))
            state.motorState[c].dq = Float(data.qvel(at: dofAddressForJoint[c]))
            state.motorState[c].tauEst = Float(lastTorques[c])
        }

        if let quatSensor {
            let v = data.sensorValues(quatSensor)
            state.imuState.quaternion = [Float(v[0]), Float(v[1]), Float(v[2]), Float(v[3])]
        }
        if let gyroSensor {
            let v = data.sensorValues(gyroSensor)
            state.imuState.gyroscope = [Float(v[0]), Float(v[1]), Float(v[2])]
        }
        if let accelSensor {
            let v = data.sensorValues(accelSensor)
            state.imuState.accelerometer = [Float(v[0]), Float(v[1]), Float(v[2])]
        }

        state.footForce = footContactForces().map { Int16(clamping: Int($0)) }
        return state
    }

    /// Summed contact normal force per foot geom, in canonical leg order.
    private func footContactForces() -> [Double] {
        var forces = [Double](repeating: 0, count: footGeomIds.count)
        for contact in data.contacts(max: 128) {
            for (leg, geomId) in footGeomIds.enumerated()
            where contact.geom1 == geomId || contact.geom2 == geomId {
                forces[leg] += contact.forceNormal
            }
        }
        return forces
    }

    // MARK: - Test hooks

    func setJointPositionForTesting(mujocoIndex: Int, value: Double) {
        let joints = model.joints
        data.setQpos(at: joints[mujocoIndex + 1].qposadr, value)  // +1 skips the free joint
        mjForward(model, data)
    }

    /// Reads a joint's qpos by MuJoCo's own declaration-order index, bypassing
    /// `qposAddressForJoint` entirely. Unlike `lowState()`, which reads back
    /// through the very canonical-order array a permutation bug would live
    /// in, this gives a test an independent ground truth: MuJoCo's tree order
    /// is fixed by the model XML, not by anything `Go2JointMap` computes.
    func jointPositionForTesting(mujocoIndex: Int) -> Double {
        let joints = model.joints
        return data.qpos(at: joints[mujocoIndex + 1].qposadr)  // +1 skips the free joint
    }

    func appliedTorquesForTesting() -> [Double] { lastTorques }
}

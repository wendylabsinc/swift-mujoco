import MuJoCo
import RobotKit

public struct CartpoleObservation: ObservationEncoding {
    public let cartPosition: Double
    public let cartVelocity: Double
    public let poleAngle: Double
    public let poleAngularVelocity: Double

    public var asArray: [Float] {
        [Float(cartPosition), Float(cartVelocity), Float(poleAngle), Float(poleAngularVelocity)]
    }
}

/// Cart-on-a-rail with an unactuated pole, balanced by a continuous force on
/// the cart. Not `Sendable` (it owns `MjModel`/`MjData`, neither of which
/// is) — each parallel rollout worker in `collectEpisode` constructs its own
/// instance rather than sharing one across tasks. The per-worker state is the
/// `MjData`; the compiled `MjModel` is shared process-wide (see `sharedModel`).
///
/// Wrapper that lets one compiled `MjModel` be shared across rollout tasks.
/// `MjModel` is deliberately not `Sendable` because its lazy introspection
/// caches are mutable; `CartpoleEnv.sharedModel` pre-warms those on a single
/// thread before publishing the model, after which it is only ever read, and
/// `mj_step`/`mj_forward` take `mjModel` as `const`.
private final class SharedModel: @unchecked Sendable {
    let model: MjModel
    init(_ model: MjModel) { self.model = model }
}

public final class CartpoleEnv: Environment {
    public static let maxSteps = 500
    static let cartPositionLimit: Double = 2.4
    // 12 degrees, matching the classic cartpole task's failure threshold.
    static let poleAngleLimit: Double = 0.2094395102393195

    /// The compiled cartpole model, built once for the whole process.
    ///
    /// Compiling per env meant `MjModel.load(xml:)` wrote a UUID-named temp file,
    /// ran `mj_loadXML`, and deleted it — *per episode*. A 200-iteration,
    /// 16-episode-per-batch run did that 3,200 times to simulate 500 steps each;
    /// model compilation dominated the actual physics.
    ///
    /// Sharing is safe because `mjModel` is read-only during stepping (`mj_step`
    /// takes it `const`) — but `MjModel`'s introspection accessors populate lazy
    /// caches on first read, which *would* race across rollout tasks. So the
    /// caches are pre-warmed here, on one thread, before any env can see the
    /// model; after this initializer the object is effectively immutable.
    private static let sharedModel: SharedModel = {
        let m: MjModel
        do {
            m = try MjModel.load(xml: CartpoleEnv.xml)
        } catch {
            // The MJCF is a compile-time constant in this file, so this is a
            // programmer error, not a runtime condition — but say which one.
            preconditionFailure("CartpoleEnv's built-in MJCF failed to compile: \(error)")
        }
        _ = m.joints
        _ = m.actuators
        _ = m.sensors
        _ = m.bodyNames
        return SharedModel(m)
    }()

    /// Escape hatch for tests to assert envs really do share one compiled model.
    static var sharedModelForTesting: MjModel { sharedModel.model }

    private let model: MjModel
    private let data: MjData
    private var stepCount = 0

    /// Test-only accessors, so the sharing contract (one model, many datas) can be
    /// asserted by identity rather than inferred from timing.
    var modelForTesting: MjModel { model }
    var dataForTesting: MjData { data }

    public init() {
        self.model = Self.sharedModel.model
        self.data = MjData(model)
    }

    /// Out of bounds OR the episode's own step cap — this environment treats
    /// its step limit as part of its own termination condition, unlike
    /// `Go2Environment`, where the training harness's external `maxSteps`
    /// plays that role instead.
    public var isTerminated: Bool {
        let obs = observation()
        let outOfBounds =
            abs(obs.cartPosition) > Self.cartPositionLimit || abs(obs.poleAngle) > Self.poleAngleLimit
        return outOfBounds || stepCount >= Self.maxSteps
    }

    public func reset() -> CartpoleObservation {
        mjResetData(model, data)
        stepCount = 0
        return observation()
    }

    public func act(_ action: [Float]) -> CartpoleObservation {
        data.setCtrl([Double(action[0])])
        mjStep(model, data)
        stepCount += 1
        return observation()
    }

    private func observation() -> CartpoleObservation {
        CartpoleObservation(
            cartPosition: data.qpos(at: 0),
            cartVelocity: data.qvel(at: 0),
            poleAngle: data.qpos(at: 1),
            poleAngularVelocity: data.qvel(at: 1)
        )
    }

    // qpos[0]/qvel[0] is the slider (cart); qpos[1]/qvel[1] is the hinge
    // (pole) — MuJoCo assigns DOF addresses in kinematic-tree declaration
    // order, and the slider joint is declared before descending into the
    // pole body. State always resets to upright/centered; exploration comes
    // from the policy's Gaussian action noise, not initial-state randomization.
    private static let xml = """
        <mujoco>
          <option timestep="0.02" gravity="0 0 -9.81"/>
          <worldbody>
            <body name="cart" pos="0 0 0">
              <joint name="slider" type="slide" axis="1 0 0" range="-3 3" damping="0.1"/>
              <geom name="cart_geom" type="box" size="0.1 0.1 0.05" rgba="0.2 0.6 0.9 1" mass="1.0"/>
              <body name="pole" pos="0 0 0.05">
                <joint name="hinge" type="hinge" axis="0 1 0" damping="0.01"/>
                <geom name="pole_geom" type="capsule" fromto="0 0 0 0 0 0.5" size="0.02" mass="0.1" rgba="0.9 0.3 0.2 1"/>
              </body>
            </body>
          </worldbody>
          <actuator>
            <motor name="slide_motor" joint="slider" gear="20" ctrlrange="-1 1"/>
          </actuator>
        </mujoco>
        """
}

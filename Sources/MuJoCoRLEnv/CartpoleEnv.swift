import MuJoCo

struct CartpoleObservation {
    let cartPosition: Double
    let cartVelocity: Double
    let poleAngle: Double
    let poleAngularVelocity: Double

    var asArray: [Float] {
        [Float(cartPosition), Float(cartVelocity), Float(poleAngle), Float(poleAngularVelocity)]
    }
}

/// Cart-on-a-rail with an unactuated pole, balanced by a continuous force on
/// the cart. Not `Sendable` (it owns `MjModel`/`MjData`, neither of which
/// is) — each parallel rollout worker in `collectEpisode` constructs its own
/// instance rather than sharing one across tasks.
final class CartpoleEnv {
    static let maxSteps = 500
    static let cartPositionLimit: Double = 2.4
    // 12 degrees, matching the classic cartpole task's failure threshold.
    static let poleAngleLimit: Double = 0.2094395102393195

    private let model: MjModel
    private let data: MjData
    private var stepCount = 0

    init() {
        self.model = try! MjModel.load(xml: Self.xml)
        self.data = MjData(model)
    }

    func reset() -> CartpoleObservation {
        mjResetData(model, data)
        stepCount = 0
        return observation()
    }

    func step(action: Float) -> (observation: CartpoleObservation, reward: Float, done: Bool) {
        data.setCtrl([Double(action)])
        mjStep(model, data)
        stepCount += 1
        let obs = observation()
        let outOfBounds =
            abs(obs.cartPosition) > Self.cartPositionLimit || abs(obs.poleAngle) > Self.poleAngleLimit
        let done = outOfBounds || stepCount >= Self.maxSteps
        return (obs, 1.0, done)
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

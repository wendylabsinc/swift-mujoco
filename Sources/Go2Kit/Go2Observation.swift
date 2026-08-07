import MuJoCoRLEnv

public struct Go2Observation: ObservationEncoding {
    public let baseAngularVelocity: (Double, Double, Double)
    /// The base's linear velocity (x, y, z), expressed in the base's own
    /// local frame (same convention as `baseAngularVelocity` — see
    /// `Go2Environment.observation()` for how it's rotated out of MuJoCo's
    /// world-frame `qvel`). Not part of `asArray` — used only for reward
    /// computation (comparing against `velocityCommand`), the same way
    /// `baseHeight`/`upright` are used only for fall detection.
    public let baseLinearVelocity: (Double, Double, Double)
    public let projectedGravity: (Double, Double, Double)
    public let velocityCommand: (vx: Double, vy: Double, wz: Double)
    /// Relative to `Go2Environment.defaultPos`, one per joint in
    /// go2.robot.json's declared order.
    public let jointPositions: [Double]
    public let jointVelocities: [Double]
    /// The residual command actually applied last step (zero on the first
    /// observation after `reset()`).
    public let previousAction: [Double]
    /// Not part of `asArray` — used only for fall detection.
    public let baseHeight: Double
    public let upright: Double

    public var asArray: [Float] {
        var values: [Float] = []
        values.reserveCapacity(45)
        values.append(Float(baseAngularVelocity.0) * 0.25)
        values.append(Float(baseAngularVelocity.1) * 0.25)
        values.append(Float(baseAngularVelocity.2) * 0.25)
        values.append(Float(projectedGravity.0))
        values.append(Float(projectedGravity.1))
        values.append(Float(projectedGravity.2))
        values.append(Float(velocityCommand.vx))
        values.append(Float(velocityCommand.vy))
        values.append(Float(velocityCommand.wz))
        values.append(contentsOf: jointPositions.map { Float($0) * 1.0 })
        values.append(contentsOf: jointVelocities.map { Float($0) * 0.05 })
        values.append(contentsOf: previousAction.map { Float($0) })
        return values
    }
}

public struct Go2Command {
    /// One residual per joint (pre-`actionScale`), in go2.robot.json's
    /// declared joint order. `Go2Environment.act` scales, adds
    /// `defaultPos`, and PD-controls to torque internally.
    public let jointPositionResiduals: [Double]

    public init(jointPositionResiduals: [Double]) {
        precondition(jointPositionResiduals.count == 12)
        self.jointPositionResiduals = jointPositionResiduals
    }
}

// Sources/RobotKit/ObservationEncoder.swift

/// Turns a canonical observation into the policy's input vector.
///
/// Layout (45 values for a 12-joint quadruped), the standard legged-locomotion
/// observation:
///
/// | Range     | Contents                                  |
/// |-----------|-------------------------------------------|
/// | `0..<3`   | base angular velocity (body frame, rad/s) |
/// | `3..<6`   | projected gravity (unit vector, body frame) |
/// | `6..<9`   | commanded velocity (vx, vy, yaw rate)     |
/// | `9..<21`  | joint position minus default pose (rad)   |
/// | `21..<33` | joint velocity (rad/s)                    |
/// | `33..<45` | previous action                           |
///
/// The previous-action history is the encoder's only state, held here rather
/// than threaded through call sites so a single encoder instance behaves
/// identically across a control loop in simulation and on hardware.
public struct ObservationEncoder: Sendable {
    public static let observationSize = 45

    public let defaultPose: [Double]
    public let jointCount: Int
    public private(set) var lastAction: [Float]

    public init(defaultPose: [Double], jointCount: Int) {
        precondition(defaultPose.count == jointCount, "defaultPose must have jointCount entries")
        self.defaultPose = defaultPose
        self.jointCount = jointCount
        self.lastAction = [Float](repeating: 0, count: jointCount)
    }

    public mutating func reset() {
        lastAction = [Float](repeating: 0, count: jointCount)
    }

    public mutating func noteAction(_ action: [Float]) {
        precondition(action.count == jointCount, "action must have jointCount entries")
        lastAction = action
    }

    public mutating func encode(
        _ observation: RobotObservation, commandedVelocity: (Double, Double, Double)
    ) -> [Float] {
        precondition(observation.joints.count == jointCount, "observation joint count mismatch")
        var out = [Float]()
        out.reserveCapacity(Self.observationSize)

        let w = observation.imu.angularVelocity
        out.append(Float(w.0))
        out.append(Float(w.1))
        out.append(Float(w.2))

        let g = Self.projectedGravity(observation.imu.orientation)
        out.append(Float(g.0))
        out.append(Float(g.1))
        out.append(Float(g.2))

        out.append(Float(commandedVelocity.0))
        out.append(Float(commandedVelocity.1))
        out.append(Float(commandedVelocity.2))

        for i in 0..<jointCount {
            out.append(Float(observation.joints[i].position - defaultPose[i]))
        }
        for i in 0..<jointCount {
            out.append(Float(observation.joints[i].velocity))
        }
        out.append(contentsOf: lastAction)
        return out
    }

    /// World-frame down (0, 0, -1) expressed in the body frame: `R(q) · (0,0,-1)`,
    /// applying the quaternion-to-matrix rotation directly (not its conjugate) —
    /// the sign convention that matches `unitree_go/IMUState` orientation, where
    /// rolling 90° about x maps world -z to body +y. Upright gives (0, 0, -1);
    /// the vector tilts as the base does, which is how the policy perceives
    /// orientation without an absolute yaw reference.
    static func projectedGravity(
        _ q: (Double, Double, Double, Double)
    ) -> (Double, Double, Double) {
        let (w, x, y, z) = q
        // Rotate (0,0,-1) by q using the standard quaternion-to-matrix formula.
        let vx = -2 * (x * z + w * y)
        let vy = -2 * (y * z - w * x)
        let vz = -(1 - 2 * (x * x + y * y))
        return (vx, vy, vz)
    }
}

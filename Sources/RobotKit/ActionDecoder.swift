// Sources/RobotKit/ActionDecoder.swift

/// Turns a policy's output vector into a canonical PD command.
///
/// The action is a per-joint offset from the default pose, scaled by `scale`:
/// `position = defaultPose[i] + scale * action[i]`, with fixed `kp`/`kd` and no
/// feedforward torque. A zero action therefore holds the default pose, which is
/// what makes an untrained or zeroed policy a safe stand rather than a collapse.
public struct ActionDecoder: Sendable {
    public let defaultPose: [Double]
    public let scale: Double
    public let kp: Double
    public let kd: Double

    public init(defaultPose: [Double], scale: Double, kp: Double, kd: Double) {
        self.defaultPose = defaultPose
        self.scale = scale
        self.kp = kp
        self.kd = kd
    }

    public func decode(_ action: [Float], stamp: RobotTime) -> RobotCommand {
        precondition(action.count == defaultPose.count, "action count must match defaultPose")
        let targets = (0..<action.count).map { i in
            JointTarget(
                position: defaultPose[i] + scale * Double(action[i]),
                velocity: 0,
                feedforwardTorque: 0,
                kp: kp,
                kd: kd
            )
        }
        return RobotCommand(stamp: stamp, joints: targets)
    }
}

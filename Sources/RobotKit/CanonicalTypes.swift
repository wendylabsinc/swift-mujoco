// Sources/RobotKit/CanonicalTypes.swift

/// A timestamp from whichever clock is active — simulation or wall.
/// Callers never branch on which; they read this off the observation.
public struct RobotTime: Sendable, Equatable, Comparable {
    public var nanoseconds: UInt64

    public init(nanoseconds: UInt64) { self.nanoseconds = nanoseconds }
    public init(seconds: Double) { self.nanoseconds = UInt64((seconds * 1_000_000_000).rounded()) }

    public var seconds: Double { Double(nanoseconds) / 1_000_000_000 }

    public static func < (a: RobotTime, b: RobotTime) -> Bool { a.nanoseconds < b.nanoseconds }
}

/// One joint's measured state. Angles in radians, velocities rad/s, effort N·m.
public struct JointReading: Sendable, Equatable {
    public var position: Double
    public var velocity: Double
    public var effort: Double

    public init(position: Double, velocity: Double, effort: Double) {
        self.position = position
        self.velocity = velocity
        self.effort = effort
    }
}

/// One joint's PD setpoint. Mirrors `unitree_go/MotorCmd` exactly, which is
/// also what a MuJoCo torque actuator can be driven with once the PD law is
/// evaluated: `tau = kp*(position - q) + kd*(velocity - dq) + feedforwardTorque`.
public struct JointTarget: Sendable, Equatable {
    public var position: Double
    public var velocity: Double
    public var feedforwardTorque: Double
    public var kp: Double
    public var kd: Double

    public init(
        position: Double, velocity: Double = 0, feedforwardTorque: Double = 0,
        kp: Double = 0, kd: Double = 0
    ) {
        self.position = position
        self.velocity = velocity
        self.feedforwardTorque = feedforwardTorque
        self.kp = kp
        self.kd = kd
    }
}

/// Orientation as (w, x, y, z); angular velocity rad/s and linear acceleration
/// m/s² both in the IMU's body frame, matching `unitree_go/IMUState`.
public struct IMUReading: Sendable, Equatable {
    public var orientation: (Double, Double, Double, Double)
    public var angularVelocity: (Double, Double, Double)
    public var linearAcceleration: (Double, Double, Double)

    public init(
        orientation: (Double, Double, Double, Double),
        angularVelocity: (Double, Double, Double),
        linearAcceleration: (Double, Double, Double)
    ) {
        self.orientation = orientation
        self.angularVelocity = angularVelocity
        self.linearAcceleration = linearAcceleration
    }

    public static func == (a: IMUReading, b: IMUReading) -> Bool {
        a.orientation == b.orientation && a.angularVelocity == b.angularVelocity
            && a.linearAcceleration == b.linearAcceleration
    }
}

public struct ContactReading: Sendable, Equatable {
    public var normalForce: Double
    public var inContact: Bool

    public init(normalForce: Double, inContact: Bool) {
        self.normalForce = normalForce
        self.inContact = inContact
    }
}

/// The canonical robot state. Joint order is the Unitree/firmware order
/// (FR, FL, RR, RL — each hip, thigh, calf); contacts are in the same leg
/// order (FR, FL, RR, RL). See `Go2JointMap`.
public struct RobotObservation: Sendable, Equatable {
    public var stamp: RobotTime
    public var joints: [JointReading]
    public var imu: IMUReading
    public var contacts: [ContactReading]

    public init(
        stamp: RobotTime, joints: [JointReading], imu: IMUReading, contacts: [ContactReading]
    ) {
        self.stamp = stamp
        self.joints = joints
        self.imu = imu
        self.contacts = contacts
    }
}

/// The canonical robot command, in the same joint order as `RobotObservation`.
public struct RobotCommand: Sendable, Equatable {
    public var stamp: RobotTime
    public var joints: [JointTarget]

    public init(stamp: RobotTime, joints: [JointTarget]) {
        self.stamp = stamp
        self.joints = joints
    }
}

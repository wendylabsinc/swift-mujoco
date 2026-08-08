// Sources/RobotKitGo2/Go2Adapter.swift
import RobotKit

/// Translates between the Go2's vendor messages and canonical `RobotKit` types.
///
/// Both simulation and hardware produce `LowState` and consume `LowCmd`, so
/// this single adapter is the only place vendor semantics live — and it is
/// exercised identically in both modes.
public struct Go2Adapter: Sendable {
    /// Number of physically present joints. `LowState`/`LowCmd` carry 20 motor
    /// slots; a Go2 uses the first 12 and leaves the rest zeroed.
    public static let jointCount = 12

    public init() {}

    public func observation(from state: LowState, stamp: RobotTime) -> RobotObservation {
        let joints = (0..<Self.jointCount).map { i in
            JointReading(
                position: Double(state.motorState[i].q),
                velocity: Double(state.motorState[i].dq),
                effort: Double(state.motorState[i].tauEst)
            )
        }

        let q = state.imuState.quaternion
        let g = state.imuState.gyroscope
        let a = state.imuState.accelerometer
        let imu = IMUReading(
            orientation: (Double(q[0]), Double(q[1]), Double(q[2]), Double(q[3])),
            angularVelocity: (Double(g[0]), Double(g[1]), Double(g[2])),
            linearAcceleration: (Double(a[0]), Double(a[1]), Double(a[2]))
        )

        // foot_force is already in canonical (FR, FL, RR, RL) leg order.
        let contacts = (0..<4).map { i in
            ContactReading(
                normalForce: Double(state.footForce[i]),
                inContact: state.footForce[i] > Self.contactForceThreshold
            )
        }

        return RobotObservation(stamp: stamp, joints: joints, imu: imu, contacts: contacts)
    }

    public func lowCmd(from command: RobotCommand) -> LowCmd {
        var cmd = LowCmd()
        cmd.head = [0xFE, 0xEF]
        cmd.levelFlag = Self.lowLevelFlag

        for i in 0..<Self.jointCount {
            let target = command.joints[i]
            cmd.motorCmd[i].mode = Self.servoMode
            cmd.motorCmd[i].q = Float(target.position)
            cmd.motorCmd[i].dq = Float(target.velocity)
            cmd.motorCmd[i].tau = Float(target.feedforwardTorque)
            cmd.motorCmd[i].kp = Float(target.kp)
            cmd.motorCmd[i].kd = Float(target.kd)
        }

        cmd.crc = UnitreeCRC.crc(for: cmd)
        return cmd
    }

    /// Placeholder threshold, not a calibrated real-sensor value. The brief's
    /// own test requires force=10 to register as contact, which forced this
    /// down from the brief's originally-suggested 20 (a value chosen to
    /// reject small nonzero swing noise). At `1`, realistic swing-noise
    /// forces could register as false contact — revisit against actual Go2
    /// foot-sensor noise during hardware bring-up.
    static let contactForceThreshold: Int16 = 1
    /// `0xFF` selects low-level (direct motor) control rather than sport mode.
    static let lowLevelFlag: UInt8 = 0xFF
    /// Per-motor servo mode: honor q/dq/tau/kp/kd.
    static let servoMode: UInt8 = 0x01
}

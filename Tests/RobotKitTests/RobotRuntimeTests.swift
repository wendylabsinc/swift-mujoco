// Tests/RobotKitTests/RobotRuntimeTests.swift
import Testing

@testable import RobotKit

private let pose: [Double] = Array(repeating: 0.5, count: 12)

private struct ConstantController: Controller {
    var value: Float
    var resetCount = 0
    mutating func act(observation: [Float]) -> [Float] {
        [Float](repeating: value, count: 12)
    }
    mutating func reset() { resetCount += 1 }
}

private func sampleObservation() -> RobotObservation {
    RobotObservation(
        stamp: RobotTime(nanoseconds: 123),
        joints: pose.map { JointReading(position: $0, velocity: 0, effort: 0) },
        imu: IMUReading(
            orientation: (1, 0, 0, 0), angularVelocity: (0, 0, 0), linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 5, inContact: true), count: 4))
}

@Test func runtimeTurnsAnObservationIntoATimestampedCommand() {
    var runtime = RobotRuntime(
        controller: ConstantController(value: 0),
        encoder: ObservationEncoder(defaultPose: pose, jointCount: 12),
        decoder: ActionDecoder(defaultPose: pose, scale: 0.25, kp: 20, kd: 0.5),
        commandedVelocity: (0, 0, 0))

    let cmd = runtime.tick(observation: sampleObservation())
    #expect(cmd.stamp.nanoseconds == 123)
    #expect(cmd.joints.count == 12)
    // A zero action holds the default pose.
    #expect(abs(cmd.joints[0].position - pose[0]) < 1e-9)
    #expect(cmd.joints[0].kp == 20)
}

@Test func runtimeFeedsTheActionBackIntoTheNextObservation() {
    var runtime = RobotRuntime(
        controller: ConstantController(value: 1.0),
        encoder: ObservationEncoder(defaultPose: pose, jointCount: 12),
        decoder: ActionDecoder(defaultPose: pose, scale: 0.25, kp: 20, kd: 0.5),
        commandedVelocity: (0, 0, 0))

    _ = runtime.tick(observation: sampleObservation())
    #expect(abs(runtime.lastObservationVector[33] - 0) < 1e-6)  // first tick: no history yet
    _ = runtime.tick(observation: sampleObservation())
    #expect(abs(runtime.lastObservationVector[33] - 1.0) < 1e-6)
}

@Test func standControllerCommandsTheDefaultPose() {
    var controller = StandControllerForTesting()
    let action = controller.act(observation: [Float](repeating: 0, count: 45))
    #expect(action.count == 12)
    #expect(action.allSatisfy { $0 == 0 })
}

private struct StandControllerForTesting: Controller {
    mutating func act(observation: [Float]) -> [Float] { [Float](repeating: 0, count: 12) }
    mutating func reset() {}
}

// Tests/RobotKitSimTests/ParityTests.swift
import RobotKit
import RobotKitGo2
import SwiftROS2
import Testing

@testable import RobotKitSim

/// Serializing and deserializing a LowState the way DDS would.
private func roundTripped(_ state: LowState) throws -> LowState {
    let encoder = CDREncoder(isLegacySchema: true)
    encoder.writeEncapsulationHeader()
    try state.encode(to: encoder)
    let decoder = try CDRDecoder(data: encoder.getData(), isLegacySchema: true)
    return try LowState(from: decoder)
}

@Test func serializationDoesNotChangeTheObservation() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    let joints = [JointTarget](repeating: JointTarget(position: 0, kp: 60, kd: 2), count: 12)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))
    for _ in 0..<200 {
        sim.applyLowCmd(cmd)
        sim.step()
    }

    let state = sim.lowState()
    let adapter = Go2Adapter()
    let stamp = RobotTime(seconds: sim.time)

    let direct = adapter.observation(from: state, stamp: stamp)
    let viaWire = adapter.observation(from: try roundTripped(state), stamp: stamp)

    #expect(direct == viaWire)
}

@Test func bothPathsProduceIdenticalCommands() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    for _ in 0..<100 { sim.step() }
    let state = sim.lowState()
    let adapter = Go2Adapter()
    let stamp = RobotTime(seconds: sim.time)

    func command(from s: LowState) -> RobotCommand {
        var runtime = RobotRuntime(
            controller: StandController(),
            encoder: ObservationEncoder(
                defaultPose: Go2JointMap.defaultStandPose, jointCount: 12),
            decoder: ActionDecoder(
                defaultPose: Go2JointMap.defaultStandPose, scale: 0.25, kp: 20, kd: 0.5),
            commandedVelocity: (0, 0, 0))
        return runtime.tick(observation: adapter.observation(from: s, stamp: stamp))
    }

    let a = command(from: state)
    let b = command(from: try roundTripped(state))
    #expect(a == b)
    // And the wire form of those commands must agree bit for bit, CRC included.
    #expect(adapter.lowCmd(from: a).crc == adapter.lowCmd(from: b).crc)
}

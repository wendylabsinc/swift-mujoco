// Tests/RobotKitSimTests/Go2SimulatorTests.swift
import RobotKit
import RobotKitGo2
import Testing

@testable import RobotKitSim

@Test func simulatorReportsTwelveJointsInCanonicalOrder() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    let state = sim.lowState()
    #expect(state.motorState.count == 20)
    // Slots 12..19 are unused and stay zero.
    #expect(state.motorState[12].q == 0)
}

@Test func simulatorMapsJointsIntoFirmwareOrder() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    // Drive only MuJoCo's FR_hip (tree index 3) to a distinctive angle.
    sim.setJointPositionForTesting(mujocoIndex: 3, value: 0.42)
    let state = sim.lowState()
    // Firmware index 0 is FR_hip, so the value must surface at slot 0.
    #expect(abs(Double(state.motorState[0].q) - 0.42) < 1e-6)
    #expect(abs(Double(state.motorState[3].q)) < 1e-6)
}

@Test func pdControlDrivesJointsTowardTheirTargets() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()

    var joints = [JointTarget](repeating: JointTarget(position: 0, kp: 60, kd: 2), count: 12)
    // Canonical index 1 is FR_thigh.
    joints[1] = JointTarget(position: 0.6, kp: 60, kd: 2)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))

    for _ in 0..<500 {
        sim.applyLowCmd(cmd)
        sim.step()
    }

    let reached = Double(sim.lowState().motorState[1].q)
    #expect(abs(reached - 0.6) < 0.15)
}

@Test func zeroGainsProduceZeroTorque() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    let joints = [JointTarget](repeating: JointTarget(position: 1.0), count: 12)  // kp=kd=0
    sim.applyLowCmd(Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints)))
    #expect(sim.appliedTorquesForTesting().allSatisfy { abs($0) < 1e-12 })
}

@Test func steppingAdvancesSimulationTime() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    #expect(sim.time == 0)
    for _ in 0..<10 { sim.step() }
    #expect(abs(sim.time - 0.02) < 1e-9)
}

@Test func imuReportsUprightOrientationAndGravity() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    sim.step()
    let imu = sim.lowState().imuState
    // Spawned level: w≈1, and the accelerometer reads roughly +g on z while
    // the body is supported (MuJoCo's accelerometer includes gravity).
    #expect(abs(Double(imu.quaternion[0]) - 1.0) < 0.05)
    #expect(abs(Double(imu.accelerometer[2])) > 5.0)
}

@Test func footContactsRegisterOnceTheBodySettles() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    let joints = [JointTarget](repeating: JointTarget(position: 0, kp: 60, kd: 2), count: 12)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))
    for _ in 0..<1500 {
        sim.applyLowCmd(cmd)
        sim.step()
    }
    let forces = sim.lowState().footForce
    #expect(forces.contains { $0 > 0 })
}

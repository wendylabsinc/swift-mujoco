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

@Test func validlyStampedLowCmdPassesTheCRCCheck() throws {
    let joints = [JointTarget](repeating: JointTarget(position: 0, kp: 60, kd: 2), count: 12)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))
    #expect(Go2Simulator.hasValidCRC(cmd))
}

@Test func corruptedCRCFailsTheCheckApplyLowCmdWouldTrapOn() throws {
    let joints = [JointTarget](repeating: JointTarget(position: 0, kp: 60, kd: 2), count: 12)
    var cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))
    // Simulate a drifted/corrupted wire message: the CRC no longer matches
    // the payload. `applyLowCmd` traps on this via `preconditionFailure`,
    // which `Testing` cannot catch, so the trappable check is exercised here
    // as the boolean predicate `applyLowCmd` itself calls at its call site.
    cmd.crc ^= 0xFFFF_FFFF
    #expect(!Go2Simulator.hasValidCRC(cmd))
}

@Test func distinctPerJointTargetsSurviveTheMuJoCoUnitreePermutation() throws {
    // Every fixture elsewhere in this file is leg-symmetric (all-zero, or a
    // single joint moved against a zeroed rest), so a subtly wrong
    // MuJoCo<->Unitree joint permutation could still pass every other test:
    // both the "expected" and "actual" sides would traverse the same
    // (possibly wrong) mapping and agree by construction. Giving each of the
    // 12 canonical joints a distinct target closes half of that gap; the
    // other half is that `lowState()` reads back through the very
    // `qposAddressForJoint` array a permutation bug would live in, so a
    // round trip through canonical indices alone is self-consistent by
    // construction even if that array silently mislabels which physical leg
    // is which (verified empirically while developing this test: reordering
    // `Go2JointMap.unitreeOrder`'s names does NOT fail a canonical-index-only
    // round trip). So this test cross-checks against `jointPositionForTesting`,
    // which reads MuJoCo's own declaration-order index directly — a ground
    // truth fixed by the fixture's XML (see `QuadrupedFixture`'s doc comment:
    // legs are declared FL, FR, RL, RR) and independent of anything
    // `Go2JointMap` computes.
    //
    // Targets are picked, not `Double(i) * step`, so that every one of the 12
    // values is at least 0.4 rad from every other (checked below) while
    // staying within each joint's fixture range (hip ±1.05, thigh
    // -1.57...3.49, calf -2.72...0.1) and close enough to each joint's own
    // resting value that the PD law actually reaches it in the same number
    // of steps used elsewhere in this file. Calibrated empirically: with
    // these targets and gains, actual settling error tops out around 0.12
    // rad, comfortably inside the 0.4 rad minimum separation between any two
    // targets — a wrong permutation would show up as an error close to that
    // 0.4 rad separation, not a rounding-level discrepancy.
    let targets: [Double] = [
        -1.0, 0.9, -2.6,  // canonical 0..2: FR hip, thigh, calf
        -0.6, 1.3, -2.2,  // canonical 3..5: FL
        -0.2, 1.7, -1.8,  // canonical 6..8: RR
        0.2, 2.1, -1.4,  // canonical 9..11: RL
    ]
    for i in 0..<targets.count {
        for j in (i + 1)..<targets.count {
            #expect(abs(targets[i] - targets[j]) >= 0.4 - 1e-9)
        }
    }

    // MuJoCo declaration order (fixture: FL, FR, RL, RR, each hip/thigh/calf)
    // -> the real firmware's canonical index (FR, FL, RR, RL) for that same
    // physical joint. Derived by hand from the two orderings' documented leg
    // sequences, not from `Go2JointMap.mujocoToUnitree` — the point is to
    // check the implementation against an oracle it had no part in computing.
    let mujocoIndexToExpectedCanonical = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]

    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    let joints = targets.map { JointTarget(position: $0, kp: 60, kd: 2) }
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))

    for _ in 0..<500 {
        sim.applyLowCmd(cmd)
        sim.step()
    }

    let state = sim.lowState()
    for i in 0..<targets.count {
        let reached = Double(state.motorState[i].q)
        #expect(abs(reached - targets[i]) < 0.15, "canonical joint \(i): expected \(targets[i]), got \(reached)")
    }

    for mujocoIndex in 0..<12 {
        let expectedCanonical = mujocoIndexToExpectedCanonical[mujocoIndex]
        let reached = sim.jointPositionForTesting(mujocoIndex: mujocoIndex)
        #expect(
            abs(reached - targets[expectedCanonical]) < 0.15,
            "mujoco joint \(mujocoIndex) (expected to be canonical \(expectedCanonical)): expected \(targets[expectedCanonical]), got \(reached)"
        )
    }
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

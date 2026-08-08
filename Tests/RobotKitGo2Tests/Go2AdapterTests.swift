// Tests/RobotKitGo2Tests/Go2AdapterTests.swift
import RobotKit
import Testing

@testable import RobotKitGo2

@Test func unitreeOrderMatchesFirmwareMotorIndices() {
    // motor_crc.h: FR_0=0 FR_1=1 FR_2=2 FL_0=3 FL_1=4 FL_2=5
    //              RR_0=6 RR_1=7 RR_2=8 RL_0=9 RL_1=10 RL_2=11
    #expect(Go2JointMap.unitreeOrder.names[0] == "FR_hip_joint")
    #expect(Go2JointMap.unitreeOrder.names[3] == "FL_hip_joint")
    #expect(Go2JointMap.unitreeOrder.names[6] == "RR_hip_joint")
    #expect(Go2JointMap.unitreeOrder.names[9] == "RL_hip_joint")
}

@Test func mujocoOrderMatchesMenagerieKinematicTree() {
    // go2.xml declares FL, FR, RL, RR.
    #expect(Go2JointMap.mujocoOrder.names[0] == "FL_hip_joint")
    #expect(Go2JointMap.mujocoOrder.names[3] == "FR_hip_joint")
    #expect(Go2JointMap.mujocoOrder.names[6] == "RL_hip_joint")
    #expect(Go2JointMap.mujocoOrder.names[9] == "RR_hip_joint")
}

@Test func legOrderPermutationsAreInverses() {
    #expect(Go2JointMap.mujocoToUnitree == [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
    for i in 0..<12 {
        #expect(Go2JointMap.unitreeToMuJoCo[Go2JointMap.mujocoToUnitree[i]] == i)
    }
}

@Test func adapterReadsMotorStateIntoCanonicalOrder() {
    var state = LowState()
    // Firmware index 0 is FR_hip; give it a distinctive value.
    state.motorState[0].q = 0.11
    state.motorState[0].dq = 0.22
    state.motorState[0].tauEst = 0.33
    state.motorState[3].q = 0.44  // FL_hip
    state.imuState.quaternion = [1, 0, 0, 0]
    state.imuState.gyroscope = [0.01, 0.02, 0.03]
    state.imuState.accelerometer = [0, 0, -9.81]
    state.footForce = [10, 20, 30, 40]

    let adapter = Go2Adapter()
    let obs = adapter.observation(from: state, stamp: RobotTime(nanoseconds: 7))

    #expect(obs.stamp.nanoseconds == 7)
    #expect(obs.joints.count == 12)
    // Canonical order IS firmware order, so index 0 stays FR_hip.
    #expect(abs(obs.joints[0].position - 0.11) < 1e-6)
    #expect(abs(obs.joints[0].velocity - 0.22) < 1e-6)
    #expect(abs(obs.joints[0].effort - 0.33) < 1e-6)
    #expect(abs(obs.joints[3].position - 0.44) < 1e-6)
    #expect(abs(obs.imu.angularVelocity.1 - 0.02) < 1e-6)
    #expect(obs.contacts.count == 4)
    #expect(abs(obs.contacts[1].normalForce - 20) < 1e-6)
    #expect(obs.contacts[0].inContact == true)
}

@Test func adapterWritesCommandsToMatchingMotorSlots() {
    var joints = [JointTarget](repeating: JointTarget(position: 0), count: 12)
    joints[0] = JointTarget(position: 0.5, velocity: 0.1, feedforwardTorque: 0.2, kp: 20, kd: 0.5)
    let command = RobotCommand(stamp: RobotTime(nanoseconds: 1), joints: joints)

    let adapter = Go2Adapter()
    let cmd = adapter.lowCmd(from: command)

    #expect(abs(cmd.motorCmd[0].q - 0.5) < 1e-6)
    #expect(abs(cmd.motorCmd[0].dq - 0.1) < 1e-6)
    #expect(abs(cmd.motorCmd[0].tau - 0.2) < 1e-6)
    #expect(abs(cmd.motorCmd[0].kp - 20) < 1e-6)
    #expect(abs(cmd.motorCmd[0].kd - 0.5) < 1e-6)
    // Slots 12..19 are unused on a 12-joint Go2 and must stay zeroed.
    #expect(cmd.motorCmd[12].kp == 0)
    #expect(cmd.motorCmd[19].q == 0)
}

@Test func adapterStampsAValidCRC() {
    var joints = [JointTarget](repeating: JointTarget(position: 0), count: 12)
    joints[5] = JointTarget(position: -1.8, kp: 20, kd: 0.5)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))

    var withoutCRC = cmd
    withoutCRC.crc = 0
    #expect(cmd.crc == UnitreeCRC.crc(for: withoutCRC))
    #expect(cmd.crc != 0)
}

@Test func adapterSetsTheLowLevelControlHeader() {
    let cmd = Go2Adapter().lowCmd(
        from: RobotCommand(
            stamp: RobotTime(nanoseconds: 0),
            joints: [JointTarget](repeating: JointTarget(position: 0), count: 12)))
    #expect(cmd.head[0] == 0xFE)
    #expect(cmd.head[1] == 0xEF)
    #expect(cmd.levelFlag == 0xFF)
    // Mode 0x01 = servo/position-velocity-torque control on each used motor.
    #expect(cmd.motorCmd[0].mode == 0x01)
    #expect(cmd.motorCmd[12].mode == 0x00)
}

@Test func adapterProducesTheReferenceCRCForAKnownCommand() {
    // Golden value from the reference algorithm over the exact bytes this
    // adapter must emit: head FE EF, levelFlag FF, mode 1 on slots 0..11,
    // every float zero, slots 12..19 untouched. If the header bytes, the
    // mode, the slot count, or the packed layout drift, this fails.
    let cmd = Go2Adapter().lowCmd(
        from: RobotCommand(
            stamp: RobotTime(nanoseconds: 0),
            joints: [JointTarget](repeating: JointTarget(position: 0), count: 12)))
    #expect(cmd.crc == 0xA5B1_D12B)
}

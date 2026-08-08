import SwiftROS2
import Testing

@testable import RobotKitGo2

@Test func lowCmdTypeNameMatchesTheRosPackage() {
    #expect(LowCmd.typeInfo.typeName == "unitree_go/msg/LowCmd")
    #expect(LowState.typeInfo.typeName == "unitree_go/msg/LowState")
}

@Test func lowCmdCarriesTwentyMotorSlots() {
    let cmd = LowCmd()
    #expect(cmd.motorCmd.count == 20)
}

@Test func lowStateCarriesTwentyMotorsAndFourFootForces() {
    let state = LowState()
    #expect(state.motorState.count == 20)
    #expect(state.footForce.count == 4)
}

@Test func motorCmdRoundTripsThroughCDR() throws {
    var cmd = LowCmd()
    cmd.motorCmd[3].q = 0.9
    cmd.motorCmd[3].kp = 20
    cmd.motorCmd[3].kd = 0.5

    let encoder = CDREncoder(isLegacySchema: true)
    encoder.writeEncapsulationHeader()
    try cmd.encode(to: encoder)
    let decoder = try CDRDecoder(data: encoder.getData(), isLegacySchema: true)
    let decoded = try LowCmd(from: decoder)

    #expect(abs(decoded.motorCmd[3].q - 0.9) < 1e-6)
    #expect(abs(decoded.motorCmd[3].kp - 20) < 1e-6)
    #expect(abs(decoded.motorCmd[3].kd - 0.5) < 1e-6)
}

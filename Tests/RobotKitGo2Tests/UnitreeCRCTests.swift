// Tests/RobotKitGo2Tests/UnitreeCRCTests.swift
import Testing

@testable import RobotKitGo2

@Test func crc32CoreMatchesReferenceVectors() {
    // Golden values produced by executing the verbatim reference algorithm
    // (unitree_sdk2's crc32_core: poly 0x04C11DB7, init 0xFFFFFFFF, MSB-first,
    // non-reflected, no final XOR). These pin the algorithm exactly — a
    // standard CRC-32 implementation produces different values for all four.
    #expect(UnitreeCRC.crc32Core([0]) == 0xC704_DD7B)
    #expect(UnitreeCRC.crc32Core([0, 0]) == 0x6904_BB59)
    #expect(UnitreeCRC.crc32Core([0x1234_5678, 0x9ABC_DEF0]) == 0x7D24_A31B)
    // Word order matters: the same words swapped give a different digest.
    #expect(UnitreeCRC.crc32Core([0x9ABC_DEF0, 0x1234_5678]) == 0x44E8_FA0F)
}

@Test func crcOfAnAllZeroLowCmdMatchesTheReference() {
    // The full 812-byte packed struct, entirely zeroed, run through the
    // reference implementation. Verifies the packing and the 202-word count
    // together, not just the digest function.
    #expect(UnitreeCRC.crc(for: LowCmd()) == 0x4B5A_4880)
}

@Test func packedLowCmdIsExactly812Bytes() {
    #expect(UnitreeCRC.packedLowCmdBytes(LowCmd()).count == 812)
}

@Test func packedLowCmdPlacesMotorFieldsAtDocumentedOffsets() {
    var cmd = LowCmd()
    cmd.motorCmd[0].q = 1.0
    cmd.motorCmd[1].q = 2.0
    let bytes = UnitreeCRC.packedLowCmdBytes(cmd)

    // motor_cmd starts at 24; each entry is 36 bytes; q sits 4 bytes in
    // (after mode + 3 padding bytes).
    func float32(at offset: Int) -> Float {
        let word =
            UInt32(bytes[offset]) | (UInt32(bytes[offset + 1]) << 8)
            | (UInt32(bytes[offset + 2]) << 16) | (UInt32(bytes[offset + 3]) << 24)
        return Float(bitPattern: word)
    }
    #expect(float32(at: 24 + 4) == 1.0)
    #expect(float32(at: 24 + 36 + 4) == 2.0)
}

@Test func packedLowCmdZeroesPaddingBytes() {
    let bytes = UnitreeCRC.packedLowCmdBytes(LowCmd())
    #expect(bytes[22] == 0)
    #expect(bytes[23] == 0)
    #expect(bytes[803] == 0)
}

@Test func crcChangesWhenAnyCommandedValueChanges() {
    var a = LowCmd()
    a.motorCmd[3].q = 0.9
    var b = LowCmd()
    b.motorCmd[3].q = 0.91
    #expect(UnitreeCRC.crc(for: a) != UnitreeCRC.crc(for: b))
}

@Test func crcIgnoresTheCrcFieldItself() {
    var a = LowCmd()
    a.motorCmd[0].kp = 20
    var b = a
    b.crc = 0xDEAD_BEEF
    #expect(UnitreeCRC.crc(for: a) == UnitreeCRC.crc(for: b))
}

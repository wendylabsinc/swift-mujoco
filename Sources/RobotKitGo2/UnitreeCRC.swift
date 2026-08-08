// Sources/RobotKitGo2/UnitreeCRC.swift

/// Unitree's custom CRC-32, and the packed-struct serialization it runs over.
///
/// This is deliberately NOT the CDR wire encoding: the firmware computes the
/// checksum over the raw in-memory `LowCmd` C struct (812 bytes, natural
/// 4-byte alignment), so a byte-exact replica is required. Padding bytes are
/// part of the checksummed input and must be zero.
///
/// The algorithm is a non-reflected, MSB-first CRC-32 with polynomial
/// 0x04C11DB7 and initial value 0xFFFFFFFF, and — unlike standard CRC-32 —
/// applies no final XOR. Standard CRC-32 routines produce different values.
public enum UnitreeCRC {
    public static func crc32Core(_ words: [UInt32]) -> UInt32 {
        var crc: UInt32 = 0xFFFF_FFFF
        let polynomial: UInt32 = 0x04C1_1DB7
        for word in words {
            var xbit: UInt32 = 1 << 31
            for _ in 0..<32 {
                if crc & 0x8000_0000 != 0 {
                    crc = (crc << 1) ^ polynomial
                } else {
                    crc = crc << 1
                }
                if word & xbit != 0 {
                    crc ^= polynomial
                }
                xbit >>= 1
            }
        }
        return crc
    }

    /// The 812-byte packed `LowCmd` struct, little-endian, padding zeroed.
    public static func packedLowCmdBytes(_ cmd: LowCmd) -> [UInt8] {
        var bytes = [UInt8](repeating: 0, count: 812)

        func writeUInt8(_ value: UInt8, at offset: Int) { bytes[offset] = value }
        func writeUInt16(_ value: UInt16, at offset: Int) {
            bytes[offset] = UInt8(value & 0xFF)
            bytes[offset + 1] = UInt8((value >> 8) & 0xFF)
        }
        func writeUInt32(_ value: UInt32, at offset: Int) {
            bytes[offset] = UInt8(value & 0xFF)
            bytes[offset + 1] = UInt8((value >> 8) & 0xFF)
            bytes[offset + 2] = UInt8((value >> 16) & 0xFF)
            bytes[offset + 3] = UInt8((value >> 24) & 0xFF)
        }
        func writeFloat(_ value: Float, at offset: Int) {
            writeUInt32(value.bitPattern, at: offset)
        }

        writeUInt8(cmd.head[0], at: 0)
        writeUInt8(cmd.head[1], at: 1)
        writeUInt8(cmd.levelFlag, at: 2)
        writeUInt8(cmd.frameReserve, at: 3)
        writeUInt32(cmd.sn[0], at: 4)
        writeUInt32(cmd.sn[1], at: 8)
        writeUInt32(cmd.version[0], at: 12)
        writeUInt32(cmd.version[1], at: 16)
        writeUInt16(cmd.bandwidth, at: 20)
        // bytes 22..23: padding, already zero.

        for i in 0..<20 {
            let base = 24 + i * 36
            let motor = cmd.motorCmd[i]
            writeUInt8(motor.mode, at: base)
            // base+1 ..< base+4: padding, already zero.
            writeFloat(motor.q, at: base + 4)
            writeFloat(motor.dq, at: base + 8)
            writeFloat(motor.tau, at: base + 12)
            writeFloat(motor.kp, at: base + 16)
            writeFloat(motor.kd, at: base + 20)
            writeUInt32(motor.reserve[0], at: base + 24)
            writeUInt32(motor.reserve[1], at: base + 28)
            writeUInt32(motor.reserve[2], at: base + 32)
        }

        writeUInt8(cmd.bmsCmd.off, at: 744)
        for j in 0..<3 { writeUInt8(cmd.bmsCmd.reserve[j], at: 745 + j) }
        for j in 0..<40 { writeUInt8(cmd.wirelessRemote[j], at: 748 + j) }
        for j in 0..<12 { writeUInt8(cmd.led[j], at: 788 + j) }
        for j in 0..<2 { writeUInt8(cmd.fan[j], at: 800 + j) }
        writeUInt8(cmd.gpio, at: 802)
        // byte 803: padding, already zero.
        writeUInt32(cmd.reserve, at: 804)
        // bytes 808..811: the crc field itself, excluded from the digest.

        return bytes
    }

    /// The checksum the firmware expects in `LowCmd.crc`: every 32-bit word of
    /// the packed struct except the trailing `crc` word — 202 of 203.
    public static func crc(for cmd: LowCmd) -> UInt32 {
        let bytes = packedLowCmdBytes(cmd)
        let wordCount = (bytes.count >> 2) - 1
        var words = [UInt32]()
        words.reserveCapacity(wordCount)
        for i in 0..<wordCount {
            let b = i * 4
            words.append(
                UInt32(bytes[b]) | (UInt32(bytes[b + 1]) << 8) | (UInt32(bytes[b + 2]) << 16)
                    | (UInt32(bytes[b + 3]) << 24))
        }
        return crc32Core(words)
    }
}

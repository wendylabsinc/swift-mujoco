import CMuJoCo

public struct MjError: Error, CustomStringConvertible {
    public let message: String
    public init(_ message: String) { self.message = message }
    public var description: String { message }
}

/// The MuJoCo library version string (proves the C library links and calls).
public func mujocoVersion() -> String {
    String(cString: mj_versionString())
}

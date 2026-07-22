import Foundation

public enum WorldSim {
    /// The Sim-tab slot directory: $WENDY_WORLDSIM_DIR or /tmp/wendy-worldsim.
    public static func directory() -> URL {
        if let env = ProcessInfo.processInfo.environment["WENDY_WORLDSIM_DIR"], !env.isEmpty {
            return URL(fileURLWithPath: env, isDirectory: true)
        }
        return URL(fileURLWithPath: "/tmp/wendy-worldsim", isDirectory: true)
    }

    public static func path(_ fileName: String, in dir: URL) -> URL {
        dir.appendingPathComponent(fileName)
    }

    /// Write atomically (temp file + rename) so the renderer never reads a torn file.
    /// Foundation's `.atomic` performs exactly the temp-write-then-rename Python's
    /// `os.replace` does. Best-effort: creates the dir; failures are swallowed so a
    /// transient FS hiccup never crashes the sim loop.
    public static func writeAtomic(_ data: Data, to fileName: String, in dir: URL) {
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        try? data.write(to: path(fileName, in: dir), options: .atomic)
    }
}

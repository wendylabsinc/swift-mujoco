import Foundation

public enum WorldSim {
    /// The Sim-tab slot directory: $WENDY_WORLDSIM_DIR or /tmp/wendy-worldsim.
    public static func directory() -> URL {
        if let env = ProcessInfo.processInfo.environment["WENDY_WORLDSIM_DIR"], !env.isEmpty {
            return URL(fileURLWithPath: env, isDirectory: true)
        }
        return URL(fileURLWithPath: "/tmp/wendy-worldsim", isDirectory: true)
    }

    /// The slot name for this process: $WENDY_WORLDSIM_SLOT, or "default". Multiple
    /// swift-mujoco processes can run concurrently, each in its own slot, so
    /// wendy-worldsim-server can discover and serve them independently. Rejects values that
    /// aren't a single path segment (empty, ".", "..", or containing "/") — those can't
    /// actually be served: wendy-worldsim-server's slot scan is single-level and its route
    /// path is a fixed 3 components, so a slot like "a/b" or ".." would silently never be
    /// discovered or reachable.
    public static func slot(_ env: [String: String] = ProcessInfo.processInfo.environment) -> String {
        guard let v = env["WENDY_WORLDSIM_SLOT"], !v.isEmpty, v != ".", v != "..", !v.contains("/")
        else { return "default" }
        return v
    }

    /// This process's slot directory: `directory()/slot()`. Every WorldSim write for a
    /// single running sim belongs under one slot dir.
    public static func slotDirectory() -> URL {
        directory().appendingPathComponent(slot(), isDirectory: true)
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

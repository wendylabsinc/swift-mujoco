import Foundation
import MuJoCo

public enum Menagerie {
    public static let repoURL = "https://github.com/google-deepmind/mujoco_menagerie"
    public static let vendorDirs = ["/opt/sandbox/mujoco-menagerie"]

    public static let nameMap: [String: String] = [
        "franka_panda": "franka_emika_panda", "panda": "franka_emika_panda",
        "franka_emika_panda": "franka_emika_panda",
        "fr3": "franka_fr3", "franka_fr3": "franka_fr3",
        "go2": "unitree_go2", "unitree_go2": "unitree_go2",
        "so101": "robotstudio_so101", "so_arm101": "robotstudio_so101",
        "robotstudio_so101": "robotstudio_so101",
        "so100": "trs_so_arm100", "so_arm100": "trs_so_arm100",
        "trs_so_arm100": "trs_so_arm100",
        "spot": "boston_dynamics_spot", "boston_dynamics_spot": "boston_dynamics_spot",
    ]

    static func dirName(_ name: String) -> String { nameMap[name] ?? name }

    /// The primary robot XML for a Menagerie directory, where "shortest filename"
    /// picks the wrong one.
    ///
    /// Menagerie dirs commonly hold several non-`scene.xml` models: the robot, a
    /// stripped variant, a sub-assembly, and MJX ports. Filename length does not
    /// order those — in `franka_emika_panda`, `hand.xml` (the gripper alone) is
    /// shorter than `panda.xml`, so the heuristic loaded a bare gripper. Anything
    /// not listed here falls back to `<dirName>.xml`, then to the heuristic.
    public static let primaryRobotXML: [String: String] = [
        "franka_emika_panda": "panda.xml",
        "franka_fr3": "fr3.xml",
        "unitree_go2": "go2.xml",
        "robotstudio_so101": "so101.xml",
        "trs_so_arm100": "so_arm100.xml",
        "boston_dynamics_spot": "spot.xml",
    ]

    /// Candidate robot-XML filenames for `dir`, best first: the curated mapping,
    /// then `<dir>.xml`. Split out from `resolveModelPath` so the ordering is
    /// unit-testable without a filesystem.
    static func robotXMLCandidates(forDirNamed dir: String) -> [String] {
        var out: [String] = []
        if let mapped = primaryRobotXML[dir] { out.append(mapped) }
        let byDirName = dir + ".xml"
        if !out.contains(byDirName) { out.append(byDirName) }
        return out
    }

    /// `scene.xml` by default (floor + lights). With `robot == true`, the robot
    /// model on its own: the curated ``primaryRobotXML`` entry, else `<dir>.xml`,
    /// else — only if neither exists — the shortest non-scene, non-MJX `*.xml`.
    public static func resolveModelPath(_ name: String, searchDirs: [String],
                                        robot: Bool = false) -> String? {
        let fm = FileManager.default
        let d = dirName(name)
        for root in searchDirs {
            let base = (root as NSString).appendingPathComponent(d)
            var isDir: ObjCBool = false
            guard fm.fileExists(atPath: base, isDirectory: &isDir), isDir.boolValue else { continue }
            let xmls = ((try? fm.contentsOfDirectory(atPath: base)) ?? [])
                .filter { $0.hasSuffix(".xml") }.sorted()
            let nonScene = xmls.filter { $0 != "scene.xml" }
            if robot {
                // Curated name, then <dir>.xml, then the length heuristic as a
                // last resort — excluding MJX ports, which are JAX-targeted
                // rewrites of the same robot and not what a caller asking for
                // "the robot" wants.
                for candidate in robotXMLCandidates(forDirNamed: d) where xmls.contains(candidate) {
                    return (base as NSString).appendingPathComponent(candidate)
                }
                let plausible = nonScene.filter { !$0.hasPrefix("mjx_") && !$0.contains("_mjx") }
                if let r = (plausible.isEmpty ? nonScene : plausible)
                    .min(by: { ($0.count, $0) < ($1.count, $1) }) {
                    return (base as NSString).appendingPathComponent(r)
                }
            }
            if xmls.contains("scene.xml") {
                return (base as NSString).appendingPathComponent("scene.xml")
            }
            if let first = nonScene.first {
                return (base as NSString).appendingPathComponent(first)
            }
        }
        return nil
    }

    /// How long a `git` invocation may run before it is killed. A stalled clone
    /// (captive portal, dead mirror, no egress — the normal state of affairs on a
    /// device) used to hang `waitUntilExit()` forever with no output.
    public static let fetchTimeout: Double = 120

    /// Sparse-clone one model dir into `cacheDir`; returns the repo root path.
    ///
    /// Requires `git` on PATH and network egress. Neither is a given on a device,
    /// which is why ``load(_:searchDirs:fetch:)`` does not do this by default.
    @discardableResult
    public static func fetch(_ name: String, cacheDir: URL) throws -> URL {
        let fm = FileManager.default
        try fm.createDirectory(at: cacheDir, withIntermediateDirectories: true)
        let repo = cacheDir.appendingPathComponent("mujoco_menagerie")
        if !fm.fileExists(atPath: repo.appendingPathComponent(".git").path) {
            try run(["git", "clone", "--depth", "1", "--filter=blob:none", "--sparse",
                     repoURL, repo.path])
        }
        try run(["git", "-C", repo.path, "sparse-checkout", "add", dirName(name)])
        return repo
    }

    /// Load a Menagerie model by name.
    ///
    /// - Parameter shouldFetch: clone the model from GitHub when it is not found
    ///   locally. Defaults to **false**: fetching shells out to `git` and needs
    ///   network egress, and silently reaching for the network is the wrong
    ///   default on an edge device, where it manifests as an unexplained stall.
    ///   Opt in explicitly on a workstation.
    public static func load(_ name: String, searchDirs: [String]? = nil,
                            fetch shouldFetch: Bool = false) throws -> MjModel {
        let dirs = searchDirs ?? vendorDirs
        if let p = resolveModelPath(name, searchDirs: dirs) {
            return try MjModel.load(xmlPath: p)
        }
        if shouldFetch {
            let cache = WorldSim.directory().appendingPathComponent("menagerie-cache")
            let repo = try fetch(name, cacheDir: cache)
            if let p = resolveModelPath(name, searchDirs: [repo.path]) {
                return try MjModel.load(xmlPath: p)
            }
        }
        var hint = "Pass a raw Menagerie dir name, or load via MjModel.load(xmlPath:)."
        if !shouldFetch {
            hint += " Pass fetch: true to clone it from \(repoURL) (needs git and network)."
        }
        throw MjError("MuJoCo model '\(name)' not found in \(dirs). "
                      + "Known aliases: \(Set(nameMap.values).sorted()). " + hint)
    }

    /// Run a command, capturing stderr and enforcing ``fetchTimeout``.
    private static func run(_ args: [String]) throws {
        let p = Process()
        p.executableURL = URL(fileURLWithPath: "/usr/bin/env")
        p.arguments = args
        let errPipe = Pipe()
        p.standardError = errPipe
        p.standardOutput = Pipe()
        do {
            try p.run()
        } catch {
            throw MjError("could not run \(args.first ?? "command"): \(error)")
        }

        // Drain stderr on a background thread: git writes progress there, and a
        // full 64 KiB pipe buffer would deadlock against waitUntilExit().
        let errData = Locked(Data())
        let drain = Thread {
            let d = errPipe.fileHandleForReading.readDataToEndOfFile()
            errData.withLock { $0 = d }
        }
        drain.start()

        let deadline = Date().addingTimeInterval(fetchTimeout)
        while p.isRunning && Date() < deadline {
            Thread.sleep(forTimeInterval: 0.05)
        }
        if p.isRunning {
            p.terminate()
            Thread.sleep(forTimeInterval: 0.5)
            if p.isRunning { kill(p.processIdentifier, SIGKILL) }
            p.waitUntilExit()
            throw MjError("timed out after \(Int(fetchTimeout))s: \(args.joined(separator: " "))")
        }
        p.waitUntilExit()
        guard p.terminationStatus == 0 else {
            let stderr = errData.withLock { String(decoding: $0, as: UTF8.self) }
                .trimmingCharacters(in: .whitespacesAndNewlines)
            throw MjError("command failed (\(p.terminationStatus)): \(args.joined(separator: " "))"
                          + (stderr.isEmpty ? "" : "\n" + stderr))
        }
    }
}

/// Minimal mutex box, so the stderr drain thread and the waiting thread don't
/// race on the captured output.
private final class Locked<T>: @unchecked Sendable {
    private let lock = NSLock()
    private var value: T
    init(_ value: T) { self.value = value }
    func withLock<R>(_ body: (inout T) -> R) -> R {
        lock.lock(); defer { lock.unlock() }
        return body(&value)
    }
}

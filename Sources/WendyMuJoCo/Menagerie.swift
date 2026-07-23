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

    /// scene.xml by default (floor+lights); the shortest non-scene *.xml when robot==true.
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
            if robot, let r = nonScene.min(by: { $0.count < $1.count }) {
                return (base as NSString).appendingPathComponent(r)
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

    /// Sparse-clone one model dir into `cacheDir`; returns the repo root path.
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

    public static func load(_ name: String, searchDirs: [String]? = nil,
                            fetch shouldFetch: Bool = true) throws -> MjModel {
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
        throw MjError("MuJoCo model '\(name)' not found. Vendored: \(Set(nameMap.values).sorted()). "
                      + "Pass a raw Menagerie dir name or load via MjModel.load(xmlPath:).")
    }

    private static func run(_ args: [String]) throws {
        let p = Process()
        p.executableURL = URL(fileURLWithPath: "/usr/bin/env")
        p.arguments = args
        try p.run()
        p.waitUntilExit()
        if p.terminationStatus != 0 {
            throw MjError("command failed (\(p.terminationStatus)): \(args.joined(separator: " "))")
        }
    }
}

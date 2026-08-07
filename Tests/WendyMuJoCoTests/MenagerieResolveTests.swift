import Testing
import Foundation
@testable import WendyMuJoCo

// `resolveModelPath(robot: true)` used to pick "the shortest non-scene *.xml".
// Filename length does not order Menagerie's contents: in franka_emika_panda,
// `hand.xml` (the gripper on its own) is shorter than `panda.xml`, so asking for
// the robot loaded a bare gripper. Resolution is now curated-name first, then
// <dir>.xml, then the heuristic with MJX ports excluded.

private func fixtureRepo(_ dirs: [String: [String]]) throws -> URL {
    let root = FileManager.default.temporaryDirectory
        .appendingPathComponent("mg-\(UUID().uuidString)")
    for (dir, files) in dirs {
        let d = root.appendingPathComponent(dir)
        try FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
        for f in files {
            try Data("<mujoco/>".utf8).write(to: d.appendingPathComponent(f))
        }
    }
    return root
}

/// The real franka_emika_panda file list, which is what broke the heuristic.
private let pandaFiles = [
    "scene.xml", "panda.xml", "panda_nohand.xml", "hand.xml",
    "mjx_panda.xml", "mjx_scene.xml", "mjx_single_cube.xml",
]

@Test func robotResolutionPrefersTheCuratedNameOverShortestFilename() throws {
    let root = try fixtureRepo(["franka_emika_panda": pandaFiles])
    defer { try? FileManager.default.removeItem(at: root) }

    let path = Menagerie.resolveModelPath("panda", searchDirs: [root.path], robot: true)
    #expect(path?.hasSuffix("/franka_emika_panda/panda.xml") == true,
            "expected panda.xml, got \(path ?? "nil")")
    // The specific regression: hand.xml is shorter and must not win.
    #expect(path?.hasSuffix("hand.xml") == false)
}

@Test func robotResolutionResolvesEveryCuratedAlias() throws {
    let root = try fixtureRepo([
        "franka_emika_panda": pandaFiles,
        "franka_fr3": ["scene.xml", "fr3.xml", "hand.xml"],
        "unitree_go2": ["scene.xml", "go2.xml", "go2_mjx.xml"],
        "robotstudio_so101": ["scene.xml", "so101.xml"],
        "trs_so_arm100": ["scene.xml", "so_arm100.xml", "nofingers.xml"],
        "boston_dynamics_spot": ["scene.xml", "spot.xml", "spot_arm.xml"],
    ])
    defer { try? FileManager.default.removeItem(at: root) }

    let expected = [
        "panda": "franka_emika_panda/panda.xml",
        "franka_panda": "franka_emika_panda/panda.xml",
        "fr3": "franka_fr3/fr3.xml",
        "go2": "unitree_go2/go2.xml",
        "so101": "robotstudio_so101/so101.xml",
        "so100": "trs_so_arm100/so_arm100.xml",
        "spot": "boston_dynamics_spot/spot.xml",
    ]
    for (alias, suffix) in expected {
        let path = Menagerie.resolveModelPath(alias, searchDirs: [root.path], robot: true)
        #expect(path?.hasSuffix(suffix) == true, "\(alias) -> \(path ?? "nil"), want …/\(suffix)")
    }
}

@Test func robotResolutionFallsBackToDirNameXML() throws {
    // An uncurated directory: <dir>.xml is the Menagerie convention.
    let root = try fixtureRepo(["some_new_robot": ["scene.xml", "some_new_robot.xml", "arm.xml"]])
    defer { try? FileManager.default.removeItem(at: root) }
    let path = Menagerie.resolveModelPath("some_new_robot", searchDirs: [root.path], robot: true)
    #expect(path?.hasSuffix("/some_new_robot.xml") == true, "got \(path ?? "nil")")
}

@Test func robotResolutionExcludesMJXPortsFromTheHeuristic() throws {
    // Neither a curated name nor <dir>.xml exists, so the length heuristic runs —
    // but an MJX port is a JAX-targeted rewrite, not "the robot".
    let root = try fixtureRepo(["odd_robot": ["scene.xml", "mjx_a.xml", "walker.xml"]])
    defer { try? FileManager.default.removeItem(at: root) }
    let path = Menagerie.resolveModelPath("odd_robot", searchDirs: [root.path], robot: true)
    #expect(path?.hasSuffix("/walker.xml") == true, "got \(path ?? "nil")")
}

@Test func robotResolutionFallsBackToMJXOnlyWhenNothingElseExists() throws {
    let root = try fixtureRepo(["mjx_only": ["scene.xml", "mjx_thing.xml"]])
    defer { try? FileManager.default.removeItem(at: root) }
    let path = Menagerie.resolveModelPath("mjx_only", searchDirs: [root.path], robot: true)
    #expect(path?.hasSuffix("/mjx_thing.xml") == true, "got \(path ?? "nil")")
}

@Test func robotResolutionIsDeterministicOnTies() throws {
    // Equal-length candidates must resolve by name, not by directory-listing order.
    let root = try fixtureRepo(["ties": ["bbb.xml", "aaa.xml", "ccc.xml"]])
    defer { try? FileManager.default.removeItem(at: root) }
    for _ in 0..<5 {
        let path = Menagerie.resolveModelPath("ties", searchDirs: [root.path], robot: true)
        #expect(path?.hasSuffix("/aaa.xml") == true, "got \(path ?? "nil")")
    }
}

@Test func sceneResolutionIsUnchanged() throws {
    let root = try fixtureRepo(["franka_emika_panda": pandaFiles])
    defer { try? FileManager.default.removeItem(at: root) }
    let path = Menagerie.resolveModelPath("panda", searchDirs: [root.path])
    #expect(path?.hasSuffix("/scene.xml") == true, "got \(path ?? "nil")")
}

@Test func sceneResolutionFallsBackToAnyXMLWhenThereIsNoScene() throws {
    let root = try fixtureRepo(["noscene": ["robot.xml"]])
    defer { try? FileManager.default.removeItem(at: root) }
    let path = Menagerie.resolveModelPath("noscene", searchDirs: [root.path])
    #expect(path?.hasSuffix("/robot.xml") == true, "got \(path ?? "nil")")
}

@Test func resolutionReturnsNilForAnUnknownDirectory() throws {
    let root = try fixtureRepo(["something": ["scene.xml"]])
    defer { try? FileManager.default.removeItem(at: root) }
    #expect(Menagerie.resolveModelPath("absent", searchDirs: [root.path]) == nil)
    #expect(Menagerie.resolveModelPath("absent", searchDirs: [root.path], robot: true) == nil)
}

@Test func robotXMLCandidateOrderIsCuratedThenDirName() {
    #expect(Menagerie.robotXMLCandidates(forDirNamed: "franka_emika_panda")
            == ["panda.xml", "franka_emika_panda.xml"])
    #expect(Menagerie.robotXMLCandidates(forDirNamed: "brand_new") == ["brand_new.xml"])
    // No duplicate when the curated name already is <dir>.xml.
    #expect(Menagerie.robotXMLCandidates(forDirNamed: "spot") == ["spot.xml"])
}

@Test func loadDoesNotReachTheNetworkByDefault() throws {
    // fetch: false is the default now — an edge device has neither git nor egress,
    // and silently trying was indistinguishable from a hang. The error must say so.
    let root = try fixtureRepo(["unrelated": ["scene.xml"]])
    defer { try? FileManager.default.removeItem(at: root) }
    #expect(throws: (any Error).self) {
        _ = try Menagerie.load("definitely_not_here", searchDirs: [root.path])
    }
    do {
        _ = try Menagerie.load("definitely_not_here", searchDirs: [root.path])
    } catch {
        let message = "\(error)"
        #expect(message.contains("fetch: true"),
                "the error should point at the opt-in, got: \(message)")
    }
}

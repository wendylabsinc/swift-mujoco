import Testing
import Foundation
import MuJoCo
@testable import WendyMuJoCo

@Test func nameMapResolvesFriendlyNames() {
    #expect(Menagerie.nameMap["go2"] == "unitree_go2")
    #expect(Menagerie.nameMap["panda"] == "franka_emika_panda")
}

@Test func resolvePrefersSceneThenRobotXML() throws {
    // Build a fake vendored dir: <root>/toy/{scene.xml, toy.xml}
    let root = FileManager.default.temporaryDirectory.appendingPathComponent("men-\(UUID().uuidString)")
    let toy = root.appendingPathComponent("toy")
    try FileManager.default.createDirectory(at: toy, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: root) }
    try Data("<mujoco/>".utf8).write(to: toy.appendingPathComponent("scene.xml"))
    try Data("<mujoco/>".utf8).write(to: toy.appendingPathComponent("toy.xml"))

    let scene = Menagerie.resolveModelPath("toy", searchDirs: [root.path], robot: false)
    #expect(scene?.hasSuffix("toy/scene.xml") == true)
    let robot = Menagerie.resolveModelPath("toy", searchDirs: [root.path], robot: true)
    #expect(robot?.hasSuffix("toy/toy.xml") == true)
    #expect(Menagerie.resolveModelPath("missing", searchDirs: [root.path]) == nil)
}

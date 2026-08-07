import Testing
import Foundation
@testable import WendyMuJoCo

@Test func atomicWriteRoundTrips() throws {
    let dir = FileManager.default.temporaryDirectory
        .appendingPathComponent("ws-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: dir) }

    let payload = Data(#"{"hello":1}"#.utf8)
    WorldSim.writeAtomic(payload, to: "scene.json", in: dir)
    let read = try Data(contentsOf: WorldSim.path("scene.json", in: dir))
    #expect(read == payload)
}

@Test func directoryHonorsEnvOverride() {
    // Default path when unset is the shared slot dir.
    #expect(WorldSim.directory().path.hasSuffix("wendy-worldsim")
            || WorldSim.directory().path == ProcessInfo.processInfo.environment["WENDY_WORLDSIM_DIR"])
}

@Test func roundingMatchesPlaces() {
    #expect(mjRound(1.234567, 2) == 1.23)
    #expect(mjRound(-0.0000004, 5) == 0.0)
}

@Test func slotDefaultsToDefault() {
    #expect(WorldSim.slot([:]) == "default")
}

@Test func slotHonorsEnvOverride() {
    #expect(WorldSim.slot(["WENDY_WORLDSIM_SLOT": "cartpole"]) == "cartpole")
}

@Test func slotTreatsEmptyOverrideAsUnset() {
    #expect(WorldSim.slot(["WENDY_WORLDSIM_SLOT": ""]) == "default")
}

@Test func slotRejectsPathSeparators() {
    #expect(WorldSim.slot(["WENDY_WORLDSIM_SLOT": "a/b"]) == "default")
}

@Test func slotRejectsDotDot() {
    #expect(WorldSim.slot(["WENDY_WORLDSIM_SLOT": ".."]) == "default")
}

@Test func slotDirectoryNestsSlotUnderDirectory() {
    let expected = WorldSim.directory().appendingPathComponent(WorldSim.slot(), isDirectory: true)
    #expect(WorldSim.slotDirectory().path == expected.path)
}

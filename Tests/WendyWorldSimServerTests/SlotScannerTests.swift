import Testing
import Foundation
@testable import WendyWorldSimServer

private func makeSlot(_ root: URL, name: String, stateAge: TimeInterval,
                      fileManager: FileManager = .default) throws {
    let dir = root.appendingPathComponent(name, isDirectory: true)
    try fileManager.createDirectory(at: dir, withIntermediateDirectories: true)
    let stateFile = dir.appendingPathComponent("state.json")
    try Data("{}".utf8).write(to: stateFile)
    try fileManager.setAttributes([.modificationDate: Date().addingTimeInterval(-stateAge)],
                                  ofItemAtPath: stateFile.path)
}

private func withTempRoot(_ body: (URL) throws -> Void) throws {
    let root = FileManager.default.temporaryDirectory.appendingPathComponent("slots-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: root, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: root) }
    try body(root)
}

@Test func liveSlotsExcludesStaleHeartbeats() throws {
    try withTempRoot { root in
        try makeSlot(root, name: "fresh", stateAge: 1)
        try makeSlot(root, name: "stale", stateAge: 30)
        let slots = liveSlots(root: root, heartbeatSeconds: 5, now: Date())
        #expect(slots.map(\.name) == ["fresh"])
    }
}

@Test func liveSlotsOrdersNewestFirst() throws {
    try withTempRoot { root in
        try makeSlot(root, name: "older", stateAge: 3)
        try makeSlot(root, name: "newer", stateAge: 1)
        let slots = liveSlots(root: root, heartbeatSeconds: 5, now: Date())
        #expect(slots.map(\.name) == ["newer", "older"])
    }
}

@Test func liveSlotsIgnoresDirectoryWithNoStateFile() throws {
    try withTempRoot { root in
        try FileManager.default.createDirectory(at: root.appendingPathComponent("empty"),
                                                 withIntermediateDirectories: true)
        #expect(liveSlots(root: root, heartbeatSeconds: 5, now: Date()).isEmpty)
    }
}

@Test func sceneTitleCacheReadsOnceAndCaches() async throws {
    try withTempRoot { root in
        try Data(#"{"title":"Falling cube"}"#.utf8).write(to: root.appendingPathComponent("scene.json"))
    }
    // withTempRoot removed `root` on exit above; recreate a persistent dir for the actor test.
    let dir = FileManager.default.temporaryDirectory.appendingPathComponent("title-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: dir) }
    try Data(#"{"title":"Falling cube"}"#.utf8).write(to: dir.appendingPathComponent("scene.json"))

    let cache = SceneTitleCache()
    let first = await cache.title(forSlot: "s", in: dir)
    #expect(first == "Falling cube")

    try FileManager.default.removeItem(at: dir.appendingPathComponent("scene.json"))   // prove it's cached
    let second = await cache.title(forSlot: "s", in: dir)
    #expect(second == "Falling cube")
}

@Test func sceneTitleCacheReturnsNilWhenSceneMissing() async {
    let dir = FileManager.default.temporaryDirectory.appendingPathComponent("title-\(UUID().uuidString)")
    let cache = SceneTitleCache()
    #expect(await cache.title(forSlot: "s", in: dir) == nil)
}

import Testing
import Foundation
@testable import WendyMuJoCo

@Test func controlDefaultsWhenFileMissing() {
    let dir = FileManager.default.temporaryDirectory
        .appendingPathComponent("ctl-\(UUID().uuidString)")
    let c = readControl(in: dir)   // dir doesn't exist
    #expect(c.paused == false)
    #expect(c.step == 0 && c.reset == 0 && c.poke == 0)
    #expect(c.ctrl.isEmpty && c.qpos.isEmpty && c.qvel.isEmpty)
}

@Test func controlParsesPartialJSON() throws {
    let dir = FileManager.default.temporaryDirectory
        .appendingPathComponent("ctl-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: dir) }
    let json = #"{"paused": true, "reset": 3, "ctrl": {"mot": 1.5}}"#
    try Data(json.utf8).write(to: dir.appendingPathComponent("control.json"))
    let c = readControl(in: dir)
    #expect(c.paused == true)
    #expect(c.reset == 3)
    #expect(c.step == 0)                // absent → default
    #expect(c.ctrl["mot"] == 1.5)
    #expect(c.qpos.isEmpty)
}

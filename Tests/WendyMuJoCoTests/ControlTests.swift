import Testing
import Foundation
@testable import WendyMuJoCo

@Test func controlIsNilWhenFileMissing() throws {
    // Was `controlDefaultsWhenFileMissing`, asserting a default-valued Control.
    // That default was the bug: `Handle` fires reset/poke on counter *change*, so
    // handing back all-zero counters for an unreadable file fired a spurious
    // reset mid-run. "No file" is now distinguishable from "counters are zero".
    // See ControlFailSafeTests for the Handle-level consequences.
    let dir = FileManager.default.temporaryDirectory
        .appendingPathComponent("ctl-\(UUID().uuidString)")
    #expect(readControl(in: dir) == nil)   // dir doesn't exist
}

@Test func controlDefaultsAreStillAllZero() {
    // A freshly-constructed Control keeps its documented defaults; only the
    // *read* path changed.
    let c = Control()
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
    let c = try #require(readControl(in: dir))
    #expect(c.paused == true)
    #expect(c.reset == 3)
    #expect(c.step == 0)                // absent → default
    #expect(c.ctrl["mot"] == 1.5)
    #expect(c.qpos.isEmpty)
}

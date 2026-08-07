import Testing
import Foundation
import MuJoCo
@testable import WendyMuJoCo

// `readControl` used to return a default-valued `Control` for a missing or
// corrupt control.json. Since `Handle` fires reset/poke on *counter change* and a
// default Control has all counters at 0, one transient read failure looked like
// "the counters went backwards" and fired a spurious full `mjResetData` — mid-run,
// silently, wiping the sim. It now returns nil and `Handle` reuses the last known
// control.

private let pendulum = """
<mujoco><worldbody>
  <body name="pole" pos="0 0 1">
    <joint name="hinge" type="hinge" axis="0 1 0"/>
    <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02"/>
  </body>
</worldbody>
<actuator><motor name="mot" joint="hinge"/></actuator>
</mujoco>
"""

private func tempDir() throws -> URL {
    let d = FileManager.default.temporaryDirectory.appendingPathComponent("cfs-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
    return d
}

private func writeControlRaw(_ text: String, _ dir: URL) throws {
    try Data(text.utf8).write(to: dir.appendingPathComponent("control.json"))
}

private func removeControl(_ dir: URL) {
    try? FileManager.default.removeItem(at: dir.appendingPathComponent("control.json"))
}

@Test func readControlReturnsNilForMissingFile() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    #expect(readControl(in: dir) == nil)
}

@Test func readControlReturnsNilForCorruptJSON() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    try writeControlRaw("{ this is not json", dir)
    #expect(readControl(in: dir) == nil)
    // A truncated write — the realistic torn-file case.
    try writeControlRaw(#"{"reset": 3, "ctrl": {"mot": 0.5"#, dir)
    #expect(readControl(in: dir) == nil)
}

@Test func readControlDecodesValidJSON() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    try writeControlRaw(#"{"paused": true, "reset": 7, "ctrl": {"mot": 0.25}}"#, dir)
    let c = readControl(in: dir)
    #expect(c?.paused == true)
    #expect(c?.reset == 7)
    #expect(c?.ctrl["mot"] == 0.25)
}

@Test func readControlToleratesWrongFieldTypesWithoutFailingTheDocument() throws {
    // A single bad field falls back to its default; the rest of the document is
    // still honoured. That is a different situation from an unparseable file, and
    // must not be conflated with it.
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    try writeControlRaw(#"{"paused": "yes", "reset": 4}"#, dir)
    let c = readControl(in: dir)
    #expect(c != nil, "a wrong-typed field must not make the whole file unreadable")
    #expect(c?.paused == false)
    #expect(c?.reset == 4)
}

@Test func transientControlReadFailureDoesNotResetTheSim() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    // No throttling, so each sync() re-reads control.json.
    let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    // Advance the reset counter once, legitimately.
    try writeControlRaw(#"{"reset": 1}"#, dir)
    h.sync()
    #expect(d.time == 0)

    // Run the sim forward so a spurious reset is detectable.
    for _ in 0..<20 { mjStep(m, d) }
    let advanced = d.time
    #expect(advanced > 0)

    // Now the file becomes unreadable. This is the regression: `reset` read back
    // as 0 while resetSeen was 1, so applyReset fired.
    try writeControlRaw("{ truncated", dir)
    h.sync()
    #expect(d.time == advanced, "a corrupt control.json must not reset the sim")

    // And a disappearing file is equally harmless.
    removeControl(dir)
    h.sync()
    #expect(d.time == advanced, "a missing control.json must not reset the sim")
}

@Test func transientControlReadFailureKeepsCtrlSetpoints() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    try writeControlRaw(#"{"ctrl": {"mot": 0.7}}"#, dir)
    h.sync()
    #expect(d.ctrl[0] == 0.7)

    // Setpoints are reapplied from the last known control every sync, so an
    // unreadable file must not silently zero the actuator.
    d.setCtrl(0, 0)
    try writeControlRaw("{ truncated", dir)
    h.sync()
    #expect(d.ctrl[0] == 0.7, "setpoint must survive an unreadable control.json")
}

@Test func transientControlReadFailureDoesNotRefirePoke() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    try writeControlRaw(#"{"poke": 1, "qpos": {"hinge": 0.5}}"#, dir)
    h.sync()
    #expect(abs(d.qpos[0] - 0.5) < 1e-9)

    // Move away from the poked position, then break the file. A spurious poke
    // would snap the joint back to 0.5.
    d.setQpos(at: 0, 1.25)
    try writeControlRaw("{{{", dir)
    h.sync()
    #expect(abs(d.qpos[0] - 1.25) < 1e-9, "a corrupt control.json must not re-fire the poke")
}

@Test func staleCounterFromAPreviousRunDoesNotFireOnInit() throws {
    // Init baselines the counters from whatever is on disk, so a leftover
    // control.json from a previous sim must not reset the new one.
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    try writeControlRaw(#"{"reset": 42, "poke": 9, "qpos": {"hinge": 0.5}}"#, dir)
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)
    for _ in 0..<20 { mjStep(m, d) }
    let advanced = d.time
    h.sync()
    #expect(d.time == advanced, "a stale reset counter must not fire")
    #expect(abs(d.qpos[0] - 0.5) > 1e-9, "a stale poke counter must not fire")
}

@Test func closeIsObservableFromAnotherThread() throws {
    // close() has to be callable from a thread other than the sim loop, because
    // the only moment it matters is while sync() is parked in the pause loop.
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    // MjModel/MjData/Handle are deliberately non-Sendable (they wrap mutable C
    // state). Handing the handle to one worker thread and calling only close()
    // from here is exactly the crossing this test exists to cover, so opt out of
    // the check rather than weakening the types.
    nonisolated(unsafe) let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)
    #expect(h.isRunning())

    try writeControlRaw(#"{"paused": true}"#, dir)
    let unblocked = DispatchSemaphore(value: 0)
    let worker = Thread {
        h.sync()          // parks in the pause loop
        unblocked.signal()
    }
    worker.start()
    // Give the loop time to actually park, then break it from out here.
    Thread.sleep(forTimeInterval: 0.2)
    h.close()
    #expect(unblocked.wait(timeout: .now() + 5) == .success,
            "close() must unblock a paused sync()")
    #expect(!h.isRunning())
}

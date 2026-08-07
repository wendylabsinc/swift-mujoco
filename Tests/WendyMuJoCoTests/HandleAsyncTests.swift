import Testing
import Foundation
import MuJoCo
@testable import WendyMuJoCo

// Handle.sync()'s pause loop parks the calling thread with Thread.sleep. That is
// fine on a thread you own and wrong inside a Task: the cooperative pool has one
// thread per core, and a paused sim held one for the whole pause. syncAsync()
// suspends instead, and is cancellation-aware.

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
    let d = FileManager.default.temporaryDirectory.appendingPathComponent("ha-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
    return d
}

private func writeControl(_ text: String, _ dir: URL) throws {
    try Data(text.utf8).write(to: dir.appendingPathComponent("control.json"))
}

private func stateFrame(_ dir: URL) -> Int? {
    guard let data = try? Data(contentsOf: dir.appendingPathComponent("state.json")) else { return nil }
    struct Frame: Decodable { let frame: Int }
    return try? JSONDecoder().decode(Frame.self, from: data).frame
}

@Test func syncAsyncPublishesAndReturnsWhenNotPaused() async throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    mjStep(m, d)
    await h.syncAsync()
    #expect(stateFrame(dir) == 1)

    mjStep(m, d)
    await h.syncAsync()
    #expect(stateFrame(dir) == 2)
}

@Test func syncAsyncAppliesControlLikeSync() async throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    try writeControl(#"{"ctrl": {"mot": 0.7}}"#, dir)
    await h.syncAsync()
    #expect(d.ctrl[0] == 0.7)

    for _ in 0..<20 { mjStep(m, d) }
    #expect(d.time > 0)
    try writeControl(#"{"reset": 1}"#, dir)
    await h.syncAsync()
    #expect(d.time == 0)
}

@Test func syncAsyncSuspendsWhilePausedThenResumes() async throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    nonisolated(unsafe) let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    try writeControl(#"{"paused": true}"#, dir)
    let waiter = Task { await h.syncAsync() }

    // Give the pause loop time to park, then lift the pause from out here.
    try await Task.sleep(for: .milliseconds(200))
    #expect(!waiter.isCancelled)
    try writeControl(#"{"paused": false}"#, dir)

    // Must come back promptly once unpaused.
    let done = Task {
        await waiter.value
        return true
    }
    let finished = await withTaskGroup(of: Bool.self) { group -> Bool in
        group.addTask { await done.value }
        group.addTask {
            try? await Task.sleep(for: .seconds(5))
            return false
        }
        let first = await group.next() ?? false
        group.cancelAll()
        return first
    }
    #expect(finished, "syncAsync must return once the pause is lifted")
}

@Test func syncAsyncReturnsOnTaskCancellation() async throws {
    // The advantage over sync(): cancelling the surrounding task unwinds the pause
    // without needing close() from another thread.
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    nonisolated(unsafe) let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    try writeControl(#"{"paused": true}"#, dir)
    let waiter = Task { await h.syncAsync() }
    try await Task.sleep(for: .milliseconds(200))
    waiter.cancel()

    let returned = await withTaskGroup(of: Bool.self) { group -> Bool in
        group.addTask { await waiter.value; return true }
        group.addTask {
            try? await Task.sleep(for: .seconds(5))
            return false
        }
        let first = await group.next() ?? false
        group.cancelAll()
        return first
    }
    #expect(returned, "cancelling the task must unwind the pause promptly")
    // The handle itself is still running — cancellation stops *waiting*, it does
    // not close the session.
    #expect(h.isRunning())
}

@Test func syncAsyncStillHonoursCloseFromAnotherThread() async throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    nonisolated(unsafe) let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    try writeControl(#"{"paused": true}"#, dir)
    let waiter = Task { await h.syncAsync() }
    try await Task.sleep(for: .milliseconds(200))
    h.close()

    let returned = await withTaskGroup(of: Bool.self) { group -> Bool in
        group.addTask { await waiter.value; return true }
        group.addTask {
            try? await Task.sleep(for: .seconds(5))
            return false
        }
        let first = await group.next() ?? false
        group.cancelAll()
        return first
    }
    #expect(returned, "close() must unwind the async pause too")
    #expect(!h.isRunning())
}

@Test func syncAsyncAppliesResetWhilePaused() async throws {
    // Reset and poke still take effect during a pause, same as the blocking form.
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    nonisolated(unsafe) let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    for _ in 0..<20 { mjStep(m, d) }
    #expect(d.time > 0)

    try writeControl(#"{"paused": true}"#, dir)
    let waiter = Task { await h.syncAsync() }
    try await Task.sleep(for: .milliseconds(150))

    try writeControl(#"{"paused": true, "reset": 1}"#, dir)
    try await Task.sleep(for: .milliseconds(300))
    #expect(d.time == 0, "a reset issued while paused must still fire")

    h.close()
    _ = await waiter.value
}

@Test func syncAsyncSingleStepReleasesThePause() async throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    nonisolated(unsafe) let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)

    try writeControl(#"{"paused": true}"#, dir)
    let waiter = Task { await h.syncAsync() }
    try await Task.sleep(for: .milliseconds(150))

    // Advancing the step counter is the single-step gesture.
    try writeControl(#"{"paused": true, "step": 1}"#, dir)
    let returned = await withTaskGroup(of: Bool.self) { group -> Bool in
        group.addTask { await waiter.value; return true }
        group.addTask {
            try? await Task.sleep(for: .seconds(5))
            return false
        }
        let first = await group.next() ?? false
        group.cancelAll()
        return first
    }
    #expect(returned, "a step counter bump must release the pause")
}

// Many concurrent paused handles must not exhaust the cooperative pool — the
// regression this whole change exists to prevent. With Thread.sleep, N paused
// handles occupy N pool threads and this deadlocks once N exceeds the core count.
@Test func manyPausedHandlesDoNotStarveTheCooperativePool() async throws {
    let count = 64
    var dirs: [URL] = []
    var handles: [Handle] = []
    var models: [MjModel] = []
    var datas: [MjData] = []
    for _ in 0..<count {
        let dir = try tempDir()
        dirs.append(dir)
        try writeControl(#"{"paused": true}"#, dir)
        let m = try MjModel.load(xml: pendulum)
        let d = MjData(m)
        models.append(m); datas.append(d)
        handles.append(Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil))
    }
    defer { for dir in dirs { try? FileManager.default.removeItem(at: dir) } }

    nonisolated(unsafe) let hs = handles
    let waiters = (0..<count).map { i in Task { await hs[i].syncAsync() } }
    try await Task.sleep(for: .milliseconds(300))

    // A fresh task must still get scheduled promptly even with 64 paused sims.
    let scheduled = await withTaskGroup(of: Bool.self) { group -> Bool in
        group.addTask { true }
        group.addTask {
            try? await Task.sleep(for: .seconds(5))
            return false
        }
        let first = await group.next() ?? false
        group.cancelAll()
        return first
    }
    #expect(scheduled, "paused handles must not occupy cooperative-pool threads")

    for h in hs { h.close() }
    for w in waiters { _ = await w.value }
}

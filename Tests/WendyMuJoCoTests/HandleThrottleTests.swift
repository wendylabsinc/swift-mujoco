import Testing
import Foundation
import MuJoCo
@testable import WendyMuJoCo

// `Handle.sync()` used to JSON-encode and atomically write state.json, and read
// and JSON-decode control.json, on *every call* — i.e. at whatever rate the sim
// loop steps, commonly 200 Hz. That is thousands of temp-file create+rename pairs
// a second for a UI that renders at display rate. Publishing and polling are now
// rate-limited, with an injectable clock so the throttle is testable without
// sleeping.

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

/// A hand-cranked clock so throttle boundaries are exact instead of timing-dependent.
private final class FakeClock: @unchecked Sendable {
    private let lock = NSLock()
    private var t: Double = 0
    func advance(by dt: Double) { lock.lock(); t += dt; lock.unlock() }
    func read() -> Double { lock.lock(); defer { lock.unlock() }; return t }
}

private func tempDir() throws -> URL {
    let d = FileManager.default.temporaryDirectory.appendingPathComponent("ht-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
    return d
}

private func stateFrame(_ dir: URL) throws -> Int? {
    let url = dir.appendingPathComponent("state.json")
    guard let data = try? Data(contentsOf: url) else { return nil }
    struct Frame: Decodable { let frame: Int }
    return try JSONDecoder().decode(Frame.self, from: data).frame
}

@Test func stateIsPublishedOnFirstSync() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let clock = FakeClock()
    let h = Handle(model: m, data: d, dir: dir, stateHz: 30, controlHz: 20,
                   now: { clock.read() })
    mjStep(m, d)
    h.sync()
    #expect(try stateFrame(dir) == 1)
}

@Test func stateIsThrottledWithinTheInterval() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let clock = FakeClock()
    let h = Handle(model: m, data: d, dir: dir, stateHz: 30, controlHz: 20,
                   now: { clock.read() })

    h.sync()                                   // t=0: publishes frame 1
    #expect(try stateFrame(dir) == 1)

    // 200 Hz sim loop: five more steps inside one 1/30 s window must not publish.
    for _ in 0..<5 {
        clock.advance(by: 1.0 / 200)
        mjStep(m, d)
        h.sync()
    }
    #expect(try stateFrame(dir) == 1, "throttled syncs must not publish a new frame")

    // Crossing the interval publishes exactly once more.
    clock.advance(by: 1.0 / 30)
    h.sync()
    #expect(try stateFrame(dir) == 2)
}

@Test func throttleRatioMatchesTheConfiguredRates() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let clock = FakeClock()
    let h = Handle(model: m, data: d, dir: dir, stateHz: 30, controlHz: 20,
                   now: { clock.read() })

    // One simulated second of a 200 Hz loop.
    for _ in 0..<200 {
        mjStep(m, d)
        h.sync()
        clock.advance(by: 1.0 / 200)
    }
    let frames = try stateFrame(dir) ?? 0
    // ~30 publishes for 200 syncs, not 200. Allow a little slack for where the
    // boundaries land relative to the step grid.
    #expect(frames >= 28 && frames <= 32, "expected ~30 publishes in 1s, got \(frames)")
}

@Test func nilRatesPublishEverySyncForBackwardCompatibility() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir, stateHz: nil, controlHz: nil)
    for i in 1...5 {
        mjStep(m, d)
        h.sync()
        #expect(try stateFrame(dir) == i)
    }
}

@Test func controlPollIsThrottledButResetStillLandsOnTheNextPoll() throws {
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let clock = FakeClock()
    let h = Handle(model: m, data: d, dir: dir, stateHz: 30, controlHz: 20,
                   now: { clock.read() })
    h.sync()                                   // t=0: first control poll

    for _ in 0..<20 { mjStep(m, d) }
    let advanced = d.time
    #expect(advanced > 0)

    try Data(#"{"reset": 1}"#.utf8).write(to: dir.appendingPathComponent("control.json"))

    // Inside the poll interval the new file is not read yet.
    clock.advance(by: 1.0 / 200)
    h.sync()
    #expect(d.time == advanced, "control.json is only re-read once per poll interval")

    // Crossing the interval picks it up.
    clock.advance(by: 1.0 / 20)
    h.sync()
    #expect(d.time == 0, "reset must land on the next control poll")
}

@Test func pausePublishesTheCurrentFrameBeforeParking() throws {
    // While paused there are no further frames, so the UI must be showing the
    // current one — the throttle must not swallow the last publish.
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let clock = FakeClock()
    // Poll control fast (100 Hz) and publish slowly (5 Hz), so the clock can be
    // advanced far enough to pick up the pause without also releasing the state
    // throttle. Then only a *forced* publish can produce frame 2.
    nonisolated(unsafe) let h = Handle(model: m, data: d, dir: dir, stateHz: 5, controlHz: 100,
                                       now: { clock.read() })
    h.sync()                                   // frame 1
    #expect(try stateFrame(dir) == 1)

    try Data(#"{"paused": true}"#.utf8).write(to: dir.appendingPathComponent("control.json"))
    clock.advance(by: 1.0 / 100)               // control poll yes, state publish no
    mjStep(m, d)

    let done = DispatchSemaphore(value: 0)
    let worker = Thread { h.sync(); done.signal() }
    worker.start()
    Thread.sleep(forTimeInterval: 0.3)
    #expect(try stateFrame(dir) == 2, "the frame shown at pause must be published")
    h.close()
    #expect(done.wait(timeout: .now() + 5) == .success)
}

@Test func handleAcceptsValidRates() throws {
    // Non-positive rates are precondition failures, which cannot be exercised
    // in-process without killing the runner; this just pins that the guarded
    // constructor still accepts the valid range.
    let dir = try tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum)
    let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir, stateHz: 1, controlHz: 1)
    #expect(h.isRunning())
}

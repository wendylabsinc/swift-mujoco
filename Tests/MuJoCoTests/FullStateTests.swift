import Testing
import CMuJoCo
@testable import MuJoCo

// `setFullState` used to hand a caller-sized array to `mj_setState`, which reads
// exactly `mj_stateSize(m, mjSTATE_FULLPHYSICS)` doubles unconditionally — so a
// short array (a truncated transfer, or a state captured from a *different*
// model) made it read past the end of the Swift buffer. It now traps, and
// `fullStateSize` is public so callers can check first.
//
// The trap itself is not directly testable (a precondition failure kills the
// test process), so these tests pin the size contract that makes the trap
// reachable and correct, plus the round-trip behaviour it must not break.

@Test func fullStateSizeMatchesMuJoCo() throws {
    for xml in [Fixtures.pendulum, Fixtures.boxScene, Fixtures.sensorScene] {
        let m = try MjModel.load(xml: xml)
        let d = MjData(m)
        let expected = Int(mj_stateSize(m.ptr, Int32(mjSTATE_FULLPHYSICS.rawValue)))
        #expect(d.fullStateSize == expected)
        #expect(d.getFullState().count == expected)
    }
}

@Test func fullStateSizeDiffersBetweenModels() throws {
    // The hazard the precondition guards: a state vector is only meaningful for
    // the model it came from, and these two models disagree on its length, so
    // cross-feeding them would have read out of bounds.
    let pendulum = MjData(try MjModel.load(xml: Fixtures.pendulum))
    let box = MjData(try MjModel.load(xml: Fixtures.boxScene))
    #expect(pendulum.fullStateSize != box.fullStateSize)
    #expect(pendulum.getFullState().count < box.getFullState().count)
}

@Test func fullStateRoundTripsAfterStepping() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<100 { mjStep(m, d) }
    let saved = d.getFullState()
    let savedTime = d.time
    let savedQpos = d.qpos

    for _ in 0..<50 { mjStep(m, d) }
    #expect(d.time != savedTime)

    d.setFullState(saved)
    #expect(d.time == savedTime)
    for (i, q) in savedQpos.enumerated() {
        #expect(abs(d.qpos(at: i) - q) < 1e-12)
    }
}

@Test func fullStateSizeIsAValidBufferSizeForSetFullState() throws {
    // Sizing a fresh buffer from `fullStateSize` must be accepted — this is the
    // documented way for a caller to build a state vector by hand.
    let m = try MjModel.load(xml: Fixtures.pendulum)
    let d = MjData(m)
    let zeros = [Double](repeating: 0, count: d.fullStateSize)
    d.setFullState(zeros)
    #expect(d.time == 0)
}

@Test func contactsTruncationIsReportable() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<600 { mjStep(m, d) }   // let the cube land
    // With a cap below the real contact count, the caller can now tell.
    #expect(d.ncon > 0, "expected the cube to be resting on the floor")
    #expect(d.contactsTruncated(max: 0))
    #expect(d.contacts(max: 0).isEmpty)
    #expect(!d.contactsTruncated(max: d.ncon))
    #expect(d.contacts(max: d.ncon).count == d.ncon)
}

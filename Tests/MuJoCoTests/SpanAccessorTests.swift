import Testing
@testable import MuJoCo

// The Span accessors must agree element-for-element with both the allocating
// accessors and the closure-based ones they sit alongside. This test target does
// NOT enable the `Lifetimes` experimental feature — consuming a Span needs no
// flag, and that fact is worth pinning: if it ever stops being true, this file
// fails to compile and tells us.

/// Sums a span the only way 6.3 allows — Span is not a Collection, so there is no
/// `reduce`; iterate `indices`.
private func sum(_ s: Span<Double>) -> Double {
    var total = 0.0
    for i in s.indices { total += s[i] }
    return total
}

private func elements(_ s: Span<Double>) -> [Double] {
    var out = [Double]()
    out.reserveCapacity(s.count)
    for i in s.indices { out.append(s[i]) }
    return out
}

@Test func spanAccessorsMatchTheAllocatingOnes() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<60 { mjStep(m, d) }

    #expect(elements(d.qposSpan) == d.qpos)
    #expect(elements(d.qvelSpan) == d.qvel)
    #expect(elements(d.qaccSpan) == d.qacc)
    #expect(elements(d.ctrlSpan) == d.ctrl)
}

@Test func spanAccessorsMatchTheClosureAccessors() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<60 { mjStep(m, d) }

    #expect(elements(d.qposSpan) == d.withQpos { Array($0) })
    #expect(elements(d.qvelSpan) == d.withQvel { Array($0) })
    #expect(elements(d.qaccSpan) == d.withQacc { Array($0) })
    #expect(elements(d.ctrlSpan) == d.withCtrl { Array($0) })
}

@Test func spanCountsMatchTheModelDimensions() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    #expect(d.qposSpan.count == m.nq)
    #expect(d.qvelSpan.count == m.nv)
    #expect(d.qaccSpan.count == m.nv)
    #expect(d.ctrlSpan.count == m.nu)
    #expect(d.sensordataSpan.count == m.nsensordata)
}

@Test func spanSeesMutationsThroughTheSameData() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)
    let d = MjData(m)
    d.setQpos(at: 0, 0.75)
    #expect(abs(d.qposSpan[0] - 0.75) < 1e-15)
    d.setQpos(at: 0, -0.25)
    #expect(abs(d.qposSpan[0] - (-0.25)) < 1e-15)
}

@Test func sensorSpanSlicesOutOneSensor() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    let d = MjData(m)
    mjForward(m, d)

    #expect(!m.sensors.isEmpty)
    for info in m.sensors {
        let viaSpan = elements(d.sensorValuesSpan(info))
        let viaClosure = d.withSensorValues(info) { Array($0) }
        #expect(viaSpan == viaClosure, "sensor \(info.name)")
        #expect(viaSpan.count == info.dim)
        // And it lines up with the concatenated vector at the sensor's address.
        let all = d.sensordata
        #expect(viaSpan == Array(all[info.adr..<(info.adr + info.dim)]))
    }
}

@Test func sensorSpanTracksSteppedPhysics() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    let d = MjData(m)
    let acc = try #require(m.sensor(named: "acc"))
    mjForward(m, d)
    let before = elements(d.sensorValuesSpan(acc))
    for _ in 0..<40 { mjStep(m, d) }
    let after = elements(d.sensorValuesSpan(acc))
    #expect(before.count == after.count)
    let total = sum(d.sensordataSpan)
    #expect(total.isFinite)
}

@Test func emptySpanForAZeroLengthVector() throws {
    // A scene with only static geometry has nq == nv == nu == 0. The accessors
    // must hand back a valid empty span, not a span over a null pointer.
    let staticOnly = """
    <mujoco><worldbody>
      <geom name="floor" type="plane" size="5 5 0.1"/>
    </worldbody></mujoco>
    """
    let m = try MjModel.load(xml: staticOnly)
    let d = MjData(m)
    #expect(m.nq == 0 && m.nv == 0 && m.nu == 0)
    // Hoisted into locals: #expect cannot introspect a property access on a
    // ~Escapable value, so `#expect(d.qposSpan.isEmpty)` does not compile.
    let qposEmpty = d.qposSpan.isEmpty
    let qvelEmpty = d.qvelSpan.isEmpty
    let ctrlEmpty = d.ctrlSpan.isEmpty
    #expect(qposEmpty)
    #expect(qvelEmpty)
    #expect(ctrlEmpty)
    #expect(d.qposSpan.count == 0)
    #expect(sum(d.qposSpan) == 0)
}

@Test func spansStayValidAcrossRepeatedAccess() throws {
    // Each access re-borrows; the underlying mjData buffer is stable for the life
    // of the MjData, so repeated reads must agree.
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<30 { mjStep(m, d) }
    let first = elements(d.qposSpan)
    for _ in 0..<10 {
        #expect(elements(d.qposSpan) == first)
    }
}

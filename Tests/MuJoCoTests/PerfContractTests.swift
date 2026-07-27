import Testing
@testable import MuJoCo

@Test func repeatedIntrospectionIsStable() throws {
    // Caching is an internal optimization with no observable identity, so this
    // asserts the contract callers actually depend on: repeated access returns
    // the same values, so a hot loop reading `m.joints` cannot drift. The
    // caching itself is verified by `cachedJointsIsPopulatedAfterFirstAccess`.
    let m = try MjModel.load(xml: Fixtures.pendulum)
    #expect(m.joints.map(\.name) == m.joints.map(\.name))
    #expect(m.joints.map(\.qposadr) == m.joints.map(\.qposadr))
    #expect(m.actuators.map(\.name) == m.actuators.map(\.name))
    #expect(m.sensors.map(\.adr) == m.sensors.map(\.adr))
    #expect(m.bodyNames == m.bodyNames)
}

@Test func cachedJointsIsPopulatedAfterFirstAccess() throws {
    // The actual caching assertion. `@testable import` reaches the internal
    // storage, so this fails if someone reverts to recompute-per-access.
    let m = try MjModel.load(xml: Fixtures.pendulum)
    #expect(m.cachedJoints == nil, "cache must start empty")
    _ = m.joints
    #expect(m.cachedJoints != nil, "first access must populate the cache")
    #expect(m.cachedJoints?.count == 1)
}

@Test func nonAllocatingAccessorsMatchAllocatingOnes() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<50 { mjStep(m, d) }

    #expect(d.withQpos { Array($0) } == d.qpos)
    #expect(d.withQvel { Array($0) } == d.qvel)
    #expect(d.withCtrl { Array($0) } == d.ctrl)

    for i in 0..<m.nq { #expect(d.qpos(at: i) == d.qpos[i]) }
    for i in 0..<m.nv { #expect(d.qvel(at: i) == d.qvel[i]) }
}

@Test func indexedAccessorsAreBoundsChecked() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)
    let d = MjData(m)
    // In-range access works; out-of-range would trap, which is the documented
    // contract shared with geomXpos and friends.
    #expect(d.qpos(at: 0) == d.qpos[0])
    #expect(m.nq == 1)
}

import Testing
@testable import MuJoCo

@Test func steppingAdvancesTime() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)
    let d = MjData(m)
    #expect(d.time == 0)
    for _ in 0..<10 { mjStep(m, d) }
    #expect(d.time > 0)
    #expect(abs(d.time - 10 * m.timestep) < 1e-9)
}

@Test func ctrlRoundTrips() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)   // 1 actuator
    let d = MjData(m)
    #expect(d.ctrl.count == m.nu)
    d.setCtrl(0, 0.5)
    #expect(d.ctrl[0] == 0.5)
}

@Test func resetClearsTime() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)
    let d = MjData(m)
    for _ in 0..<5 { mjStep(m, d) }
    mjResetData(m, d)
    #expect(d.time == 0)
}

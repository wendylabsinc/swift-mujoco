import Testing
@testable import MuJoCo

@Test func fullStateSaveRestore() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)
    let d = MjData(m)
    d.setCtrl(0, 0.3)
    for _ in 0..<20 { mjStep(m, d) }
    let saved = d.getFullState()
    let qposAt20 = d.qpos
    let timeAt20 = d.time

    for _ in 0..<20 { mjStep(m, d) }
    #expect(d.time > timeAt20)

    d.setFullState(saved)
    #expect(abs(d.time - timeAt20) < 1e-9)
    for i in 0..<m.nq { #expect(abs(d.qpos[i] - qposAt20[i]) < 1e-9) }
}

@Test func contactsAppearWhenBoxLands() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)   // cube starts at z=1 above floor
    let d = MjData(m)
    var sawContact = false
    for _ in 0..<3000 {
        mjStep(m, d)
        if d.ncon > 0 { sawContact = true; break }
    }
    #expect(sawContact)
    let cs = d.contacts()
    #expect(cs.count == d.ncon || cs.count == 64)
    #expect(cs.first!.forceNormal >= 0)
}

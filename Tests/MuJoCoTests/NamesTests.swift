import Testing
@testable import MuJoCo

@Test func nameIdRoundTrip() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)
    let jid = m.id(of: objJoint, name: "hinge")
    #expect(jid == 0)
    #expect(m.name(of: objJoint, id: 0) == "hinge")
    #expect(m.id(of: objJoint, name: "does-not-exist") == nil)
}

@Test func introspectsJointsAndActuators() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)
    #expect(m.joints.count == 1)
    let j = m.joints[0]
    #expect(j.name == "hinge")
    #expect(j.limited == true)
    #expect(abs(j.range.1 - 3.14) < 1e-3)

    #expect(m.actuators.count == 1)
    let a = m.actuators[0]
    #expect(a.name == "mot")
    #expect(a.ctrlLimited == true)
    #expect(abs(a.ctrlRange.0 + 1) < 1e-6 && abs(a.ctrlRange.1 - 1) < 1e-6)
}

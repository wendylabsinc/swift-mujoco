import Testing
@testable import MuJoCo

@Test func loadsBoxSceneScalars() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    #expect(m.ngeom == 3)          // floor + box + hidden
    #expect(m.nbody == 2)          // world + cube
    #expect(m.timestep > 0)
    #expect(m.gravity.2 < 0)       // default gravity is (0,0,-9.81)
}

@Test func loadInvalidXMLThrows() {
    #expect(throws: MjError.self) {
        _ = try MjModel.load(xml: "<mujoco><worldbody><geom type=\"nonsense\"/></worldbody></mujoco>")
    }
}

import Testing
@testable import MuJoCo

@Test func buildAndCompileScene() throws {
    let spec = MjSpec(floor: true, light: true)   // floor plane geom in worldbody
    let body = spec.addBody(name: "crate", pos: [0, 0, 0.5])
    spec.addGeom(type: .box, size: [0.25, 0.25, 0.25], pos: [0, 0, 0],
                 rgba: [0.4, 0.6, 0.9, 1], toBody: body)
    let model = try spec.compile()
    #expect(model.ngeom == 2)                 // floor + crate box
    #expect(model.id(of: objBody, name: "crate") != nil)
}

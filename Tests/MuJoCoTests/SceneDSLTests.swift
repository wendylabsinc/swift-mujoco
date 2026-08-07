import Testing
@testable import MuJoCo

@Test func sceneWithSingleGeomCompiles() throws {
    let scene = Scene {
        Geom(name: "floor", type: .plane, size: [5, 5, 0.1], rgba: [0.2, 0.23, 0.28, 1])
    }
    let model = try scene.compile()

    #expect(model.ngeom == 1)
    #expect(model.geomType(0) == .plane)
    #expect(model.geomSize(0) == [5, 5, 0.1])
    #expect(model.id(of: objGeom, name: "floor") != nil)

    let rgba = model.geomRgba(0)
    let expectedRgba = [0.2, 0.23, 0.28, 1.0]
    for (a, b) in zip(rgba, expectedRgba) {
        #expect(abs(a - b) < 1e-5)
    }
}

@Test func sceneDefaultGeomParametersMatchDocumentedDefaults() throws {
    let scene = Scene {
        Geom(type: .sphere, size: [0.2, 0, 0])
    }
    let model = try scene.compile()

    #expect(model.ngeom == 1)
    let rgba = model.geomRgba(0)
    for (a, b) in zip(rgba, [0.5, 0.5, 0.5, 1.0]) {
        #expect(abs(a - b) < 1e-5)
    }
    #expect(model.geomSize(0) == [0.2, 0, 0])
}

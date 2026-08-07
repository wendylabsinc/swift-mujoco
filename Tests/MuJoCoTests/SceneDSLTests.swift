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

@Test func nestedBodyWithFreeJointAndGeom() throws {
    let scene = Scene {
        Body(name: "cube", pos: [0, 0, 2]) {
            FreeJoint()
            Geom(name: "box", type: .box, size: [0.15, 0.15, 0.15], rgba: [0.2, 0.6, 0.9, 1])
        }
    }
    let model = try scene.compile()

    #expect(model.nbody == 2)   // world + cube
    #expect(model.njnt == 1)
    let cube = try #require(model.id(of: objBody, name: "cube"))
    #expect(model.id(of: objGeom, name: "box") != nil)

    let data = MjData(model)
    mjForward(model, data)
    let p = data.bodyPos(cube)
    #expect(abs(p.x) < 1e-9 && abs(p.y) < 1e-9 && abs(p.z - 2) < 1e-9)
}

@Test func bodyWithNoTrailingClosureHasNoChildren() throws {
    // Exercises Body's default `children: () -> [MjSceneElement] = { }` —
    // a childless body (e.g. a bare attachment point).
    let scene = Scene {
        Geom(type: .plane, size: [5, 5, 0.1])
        Body(name: "anchor", pos: [1, 1, 0])
    }
    let model = try scene.compile()
    #expect(model.nbody == 2)   // world + anchor
    #expect(model.ngeom == 1)   // only the floor — anchor has no geom of its own
    #expect(model.id(of: objBody, name: "anchor") != nil)
}

@Test func twoLevelsOfNestedBodies() throws {
    let scene = Scene {
        Body(name: "outer", pos: [1, 0, 0]) {
            Body(name: "inner", pos: [0, 0, 1]) {
                Geom(type: .sphere, size: [0.1, 0, 0])
            }
        }
    }
    let model = try scene.compile()

    #expect(model.nbody == 3)   // world + outer + inner
    let inner = try #require(model.id(of: objBody, name: "inner"))
    let data = MjData(model)
    mjForward(model, data)
    let p = data.bodyPos(inner)
    // inner's local pos (0,0,1) composed with outer's world pos (1,0,0).
    #expect(abs(p.x - 1) < 1e-9 && abs(p.y) < 1e-9 && abs(p.z - 1) < 1e-9)
}

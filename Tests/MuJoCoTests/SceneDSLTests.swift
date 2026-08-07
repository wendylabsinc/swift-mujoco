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

@Test func hingeJointWithRangeAndAxis() throws {
    let scene = Scene {
        Body(name: "pole", pos: [0, 0, 1]) {
            Joint(name: "hinge", type: .hinge, axis: [0, 1, 0], range: -1.5...1.5)
            Geom(type: .capsule, size: [0.02, 0.5, 0])
        }
    }
    let model = try scene.compile()

    #expect(model.njnt == 1)
    let joint = try #require(model.joints.first { $0.name == "hinge" })
    #expect(joint.type == 3)   // mjJNT_HINGE (mjtJoint_: FREE=0, BALL=1, SLIDE=2, HINGE=3)
    #expect(joint.limited == true)
    // Range is specified in degrees; MuJoCo's compiler converts to radians
    let expectedLower = -1.5 * Double.pi / 180
    let expectedUpper = 1.5 * Double.pi / 180
    #expect(abs(joint.range.0 - expectedLower) < 1e-9)
    #expect(abs(joint.range.1 - expectedUpper) < 1e-9)
}

@Test func slideAndBallJointsCompile() throws {
    let scene = Scene {
        Body(name: "a") {
            Joint(type: .slide, axis: [1, 0, 0])
            Geom(type: .box, size: [0.1, 0.1, 0.1])
        }
        Body(name: "b") {
            Joint(type: .ball)
            Geom(type: .sphere, size: [0.1, 0, 0])
        }
    }
    let model = try scene.compile()
    #expect(model.njnt == 2)
    #expect(model.joints[0].type == 2)   // mjJNT_SLIDE
    #expect(model.joints[1].type == 1)   // mjJNT_BALL
}

@Test func siteCameraAndLightAppearInCompiledModel() throws {
    let scene = Scene {
        Body(name: "head", pos: [0, 0, 1]) {
            Geom(type: .sphere, size: [0.1, 0, 0])
            Site(name: "imu", pos: [0, 0, 0.05])
            Camera(name: "eye", pos: [0.1, 0, 0], fovy: 60)
        }
        Light(pos: [0, 0, 3])
    }
    let model = try scene.compile()

    #expect(model.nsite == 1)
    #expect(model.ncam == 1)
    #expect(model.nlight == 1)
    #expect(model.id(of: objSite, name: "imu") != nil)
    #expect(model.id(of: objCamera, name: "eye") != nil)
    #expect(abs(model.camFovy(0) - 60) < 1e-9)
}

@Test func siteAndCameraUseDocumentedDefaults() throws {
    let scene = Scene {
        Body(name: "b") {
            Geom(type: .box, size: [0.1, 0.1, 0.1])
            Site()
            Camera()
        }
    }
    let model = try scene.compile()
    #expect(model.nsite == 1)
    #expect(model.ncam == 1)
    #expect(abs(model.camFovy(0) - 45) < 1e-9)
}

@Test func optionSetsTimestep() throws {
    let scene = Scene {
        Option(timestep: 0.002)
        Geom(type: .plane, size: [5, 5, 0.1])
    }
    let model = try scene.compile()
    #expect(abs(model.timestep - 0.002) < 1e-9)
}

@Test func ifInsideSceneBuilderAddsOrOmitsAnElement() throws {
    func makeScene(includeExtra: Bool) -> Scene {
        Scene {
            Geom(type: .plane, size: [5, 5, 0.1])
            if includeExtra {
                Geom(name: "extra", type: .box, size: [0.1, 0.1, 0.1])
            }
        }
    }

    let withExtra = try makeScene(includeExtra: true).compile()
    #expect(withExtra.ngeom == 2)

    let withoutExtra = try makeScene(includeExtra: false).compile()
    #expect(withoutExtra.ngeom == 1)
}

@Test func forLoopInsideSceneBuilderAddsMultipleBodies() throws {
    let scene = Scene {
        Geom(type: .plane, size: [5, 5, 0.1])
        for i in 0..<3 {
            Body(name: "box\(i)", pos: [Double(i), 0, 1]) {
                Geom(type: .box, size: [0.1, 0.1, 0.1])
            }
        }
    }
    let model = try scene.compile()
    #expect(model.nbody == 4)   // world + 3 boxes
    #expect(model.id(of: objBody, name: "box0") != nil)
    #expect(model.id(of: objBody, name: "box2") != nil)
}

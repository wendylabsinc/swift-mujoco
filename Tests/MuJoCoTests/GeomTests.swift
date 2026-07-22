import Testing
@testable import MuJoCo

@Test func geomTypesAndVisibility() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    // geom order matches declaration: 0=floor(plane) 1=box 2=hidden
    #expect(m.geomType(0) == .plane)
    #expect(m.geomType(1) == .box)
    #expect(m.geomIsVisible(0) == true)
    #expect(m.geomIsVisible(1) == true)
    #expect(m.geomIsVisible(2) == false)   // group 3 -> hidden
    let rgba = m.geomRgba(1)
    #expect(rgba.count == 4)
    #expect(abs(rgba[2] - 0.9) < 1e-6)     // blue channel from the fixture
    #expect(m.geomSize(1).count == 3)
}

@Test func geomRgbaResolvesMaterialColor() throws {
    let m = try MjModel.load(xml: Fixtures.materialScene)
    let i = m.id(of: objGeom, name: "m")!
    let c = m.geomRgba(i)
    #expect(abs(c[0] - 0.9) < 1e-6 && abs(c[1] - 0.1) < 1e-6)
}

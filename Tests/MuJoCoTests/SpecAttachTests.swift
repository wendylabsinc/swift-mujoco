import Testing
@testable import MuJoCo

private let armXML = """
<mujoco model="arm">
  <worldbody>
    <body name="link">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 0.4" size="0.03"/>
    </body>
  </worldbody>
</mujoco>
"""

@Test func meshGeomTypeIsNotSilentlyABox() throws {
    // Regression: cGeomType used to map .mesh and .other to mjGEOM_BOX, so a
    // spec-built mesh geom compiled to a box with no warning.
    let spec = MjSpec(floor: false, light: false)
    let body = spec.addBody(name: "b", pos: [0, 0, 0])
    // A mesh geom with no mesh asset must fail compilation loudly rather than
    // silently becoming a box.
    spec.addGeom(type: .mesh, size: [0.1, 0.1, 0.1], pos: [0, 0, 0],
                 rgba: [1, 1, 1, 1], toBody: body)
    #expect(throws: MjError.self) { _ = try spec.compile() }
}

@Test func parseXMLIntoSpecThenCompile() throws {
    let spec = try MjSpec(xml: armXML)
    #expect(spec.findBodyNames().contains("link"))
    let m = try spec.compile()
    #expect(m.id(of: objBody, name: "link") != nil)
    #expect(m.njnt == 1)
}

@Test func attachWithPrefixGivesTwoIndependentCopies() throws {
    let parent = MjSpec(floor: true, light: true)
    let childA = try MjSpec(xml: armXML)
    let childB = try MjSpec(xml: armXML)

    try parent.attach(childA, prefix: "a_", toBody: "world")
    try parent.attach(childB, prefix: "b_", toBody: "world")

    let m = try parent.compile()
    // Both copies present under their prefixes, and the unprefixed name is gone.
    #expect(m.id(of: objBody, name: "a_link") != nil)
    #expect(m.id(of: objBody, name: "b_link") != nil)
    #expect(m.id(of: objBody, name: "link") == nil)
    // Two hinges, two capsules, plus the floor plane.
    #expect(m.njnt == 2)
    #expect(m.ngeom == 3)
}

// Pins the lifetime contract of `attach`: `mjs_attach` links the child by
// reference (the merge happens inside `mj_compile`, reading live from the
// child's `mjSpec`), so `MjSpec.attach` must retain the child itself. Here
// the caller's only strong reference to `child` dies at the end of the `do`
// scope, before `compile()` runs.
//
// Note: this test alone does not *prove* the retain is correct — without it,
// reading the freed child would be undefined behaviour, which can crash OR
// can silently "work" on a given run/allocator state. It is the retain in
// `attach` (an `attachedChildren: [MjSpec]` array on the parent) that makes
// this safe; this test only documents the contract and would catch an
// outright removal of that retain.
@Test func parentRetainsAttachedChildBeyondCallerScope() throws {
    let parent = MjSpec(floor: true, light: true)
    do {
        let child = try MjSpec(xml: armXML)
        try parent.attach(child, prefix: "a_", toBody: "world")
    }   // caller's only reference to child dies here
    let m = try parent.compile()
    #expect(m.id(of: objBody, name: "a_link") != nil)
}

@Test func addSiteAndCameraAppearInCompiledModel() throws {
    let spec = MjSpec(floor: false, light: true)
    let body = spec.addBody(name: "head", pos: [0, 0, 1])
    spec.addSite(name: "imu", pos: [0, 0, 0], toBody: body)
    spec.addCamera(name: "eye", pos: [0.1, 0, 0], fovy: 60, toBody: body)
    let m = try spec.compile()
    #expect(m.nsite == 1)
    #expect(m.ncam == 1)
    #expect(m.id(of: objSite, name: "imu") != nil)
    #expect(m.id(of: objCamera, name: "eye") != nil)
    #expect(abs(m.camFovy(0) - 60) < 1e-9)
}

@Test func saveXMLRoundTrips() throws {
    let spec = try MjSpec(xml: armXML)
    let xml = try spec.saveXML()
    #expect(xml.contains("link"))
    // The saved XML must itself be loadable.
    let m = try MjModel.load(xml: xml)
    #expect(m.id(of: objBody, name: "link") != nil)
}

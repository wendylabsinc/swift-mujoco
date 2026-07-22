import Testing
@testable import MuJoCo

@Test func meshBuffersExtractable() throws {
    let m = try MjModel.load(xml: Fixtures.meshScene)
    #expect(m.nmesh == 1)
    #expect(m.geomType(0) == .mesh)
    let meshId = m.geomDataid(0)
    #expect(meshId >= 0)
    let verts = m.meshVertices(meshId)
    let faces = m.meshFaces(meshId)
    #expect(verts.count == 4 * 3)      // 4 inline vertices
    #expect(faces.count % 3 == 0)      // triangles
    #expect(faces.count >= 3)          // convex hull built by MuJoCo
    #expect(m.meshName(meshId) == "tri")
}

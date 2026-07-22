import Testing
import Foundation
import MuJoCo
@testable import WendyMuJoCo

private let boxScene = """
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body name="cube" pos="0 0 1">
      <freejoint/>
      <geom name="box" type="box" size="0.1 0.1 0.1" rgba="0.1 0.5 0.9 1" group="0"/>
      <geom name="hidden" type="box" size="0.1 0.1 0.1" group="3"/>
    </body>
  </worldbody>
</mujoco>
"""

private let meshScene = """
<mujoco>
  <asset><mesh name="tri" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/></asset>
  <worldbody>
    <geom name="a" type="mesh" mesh="tri" rgba="1 0 0 1"/>
    <body pos="1 0 0"><geom name="b" type="mesh" mesh="tri" rgba="0 1 0 1"/></body>
  </worldbody>
</mujoco>
"""

@Test func sceneListsOnlyVisibleGeomsWithTrueIndices() throws {
    let m = try MjModel.load(xml: boxScene)
    let scene = buildScene(m, title: "t")
    #expect(scene.up == "z")
    #expect(scene.engine == "mujoco")
    // floor(0) + box(1) visible; hidden(2) excluded
    #expect(scene.geoms.map(\.i) == [0, 1])
    #expect(scene.geoms[0].type == "plane")
    #expect(scene.geoms[1].type == "box")
    #expect(scene.geoms[1].rgba.count == 4)
    #expect(scene.geoms.allSatisfy { $0.mesh == nil })   // no mesh geoms here
    #expect(scene.meshes.isEmpty)
}

@Test func sceneEncodesToExpectedJSONKeys() throws {
    let m = try MjModel.load(xml: boxScene)
    let data = try JSONEncoder().encode(buildScene(m, title: "t"))
    let obj = try JSONSerialization.jsonObject(with: data) as! [String: Any]
    #expect(Set(obj.keys) == ["title", "up", "engine", "geoms", "meshes"])
    let g0 = (obj["geoms"] as! [[String: Any]])[0]
    #expect(Set(g0.keys) == ["i", "type", "size", "rgba"])   // no "mesh" key when nil
}

@Test func sceneDeduplicatesMeshAndCarriesMeshKey() throws {
    let m = try MjModel.load(xml: meshScene)
    let scene = buildScene(m, title: "t")
    // both geoms share mesh "tri" -> a single meshes entry
    #expect(scene.meshes.count == 1)
    #expect(scene.meshes["tri"] != nil)
    #expect(scene.meshes["tri"]!.vert.count == 12)   // 4 verts * 3
    let meshGeoms = scene.geoms.filter { $0.mesh != nil }
    #expect(meshGeoms.count == 2)
    #expect(meshGeoms.allSatisfy { $0.mesh == "tri" })
    // and the mesh KEY is actually present in the encoded JSON for a mesh geom
    let data = try JSONEncoder().encode(scene)
    let obj = try JSONSerialization.jsonObject(with: data) as! [String: Any]
    let g = (obj["geoms"] as! [[String: Any]]).first { $0["type"] as? String == "mesh" }!
    #expect(g["mesh"] as? String == "tri")
}

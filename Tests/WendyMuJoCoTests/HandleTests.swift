import Testing
import Foundation
import MuJoCo
@testable import WendyMuJoCo

private let pendulum = """
<mujoco><worldbody>
  <body name="pole" pos="0 0 1">
    <joint name="hinge" type="hinge" axis="0 1 0"/>
    <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02"/>
  </body>
</worldbody>
<actuator><motor name="mot" joint="hinge"/></actuator>
</mujoco>
"""

private func tempDir() -> URL {
    let d = FileManager.default.temporaryDirectory.appendingPathComponent("h-\(UUID().uuidString)")
    try! FileManager.default.createDirectory(at: d, withIntermediateDirectories: true)
    return d
}
private func writeControl(_ json: String, _ dir: URL) {
    try! Data(json.utf8).write(to: dir.appendingPathComponent("control.json"))
}

@Test func initWritesSceneAndSyncWritesState() throws {
    let dir = tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum); let d = MjData(m)
    let h = Handle(model: m, data: d, title: "pend", dir: dir)
    #expect(FileManager.default.fileExists(atPath: dir.appendingPathComponent("scene.json").path))
    mjStep(m, d); h.sync()
    let st = try JSONSerialization.jsonObject(
        with: Data(contentsOf: dir.appendingPathComponent("state.json"))) as! [String: Any]
    #expect(st["frame"] as? Int == 1)
    #expect((st["pose"] as! [[Double]]).count == m.ngeom)
}

@Test func resetCounterResetsData() throws {
    let dir = tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum); let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir)
    for _ in 0..<20 { mjStep(m, d) }
    #expect(d.time > 0)
    writeControl(#"{"reset": 1}"#, dir)
    h.sync()                       // sees reset counter advance 0 -> 1
    #expect(d.time == 0)
}

@Test func ctrlSetpointApplied() throws {
    let dir = tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum); let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir)
    writeControl(#"{"ctrl": {"mot": 0.7}}"#, dir)
    h.sync()
    #expect(d.ctrl[0] == 0.7)      // resolved actuator "mot" -> index 0
}

@Test func pokeSetsJointPosition() throws {
    let dir = tempDir(); defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: pendulum); let d = MjData(m)
    let h = Handle(model: m, data: d, dir: dir)
    writeControl(#"{"poke": 1, "qpos": {"hinge": 0.5}}"#, dir)
    h.sync()
    #expect(abs(d.qpos[0] - 0.5) < 1e-9)
}

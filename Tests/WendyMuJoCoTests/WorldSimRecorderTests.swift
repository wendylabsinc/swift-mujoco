import Testing
import Foundation
import MuJoCo
@testable import WendyMuJoCo

private let boxScene = """
<mujoco><worldbody>
  <geom name="floor" type="plane" size="5 5 0.1"/>
  <body name="cube" pos="0 0 1"><freejoint/>
    <geom name="box" type="box" size="0.1 0.1 0.1"/>
  </body>
</worldbody></mujoco>
"""

@Test func recorderWritesSceneOnceAndStateOnEveryCall() throws {
    let dir = FileManager.default.temporaryDirectory.appendingPathComponent("wsr-\(UUID().uuidString)")
    defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: boxScene)
    let d = MjData(m)
    mjForward(m, d)
    var recorder = WorldSimRecorder(dir: dir)

    recorder.record(model: m, data: d, title: "first", frame: 0)
    recorder.record(model: m, data: d, title: "second", frame: 1)   // title change ignored: scene already written

    let scene = try Data(contentsOf: WorldSim.path("scene.json", in: dir))
    let sceneObj = try JSONSerialization.jsonObject(with: scene) as! [String: Any]
    #expect(sceneObj["title"] as? String == "first")   // written once, on the first call

    let state = try Data(contentsOf: WorldSim.path("state.json", in: dir))
    let stateObj = try JSONSerialization.jsonObject(with: state) as! [String: Any]
    #expect(stateObj["frame"] as? Int == 1)            // state.json reflects the latest record()
}

@Test func recorderPassesHudAndLevelThrough() throws {
    let dir = FileManager.default.temporaryDirectory.appendingPathComponent("wsr-\(UUID().uuidString)")
    defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: boxScene)
    let d = MjData(m)
    mjForward(m, d)
    var recorder = WorldSimRecorder(dir: dir)

    recorder.record(model: m, data: d, title: "t", frame: 3, hud: ["gate": .text("2/5")], level: 2)

    let state = try Data(contentsOf: WorldSim.path("state.json", in: dir))
    let obj = try JSONSerialization.jsonObject(with: state) as! [String: Any]
    #expect((obj["hud"] as! [String: Any])["gate"] as? String == "2/5")
    #expect(obj["level"] as? Int == 2)
}

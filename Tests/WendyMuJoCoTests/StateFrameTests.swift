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

@Test func statePoseCoversEveryGeom() throws {
    let m = try MjModel.load(xml: boxScene)
    let d = MjData(m)
    mjForward(m, d)
    let s = buildState(m, d, frame: 7, hud: ["gate": .text("1/5"), "alt": .number(1.0)],
                       level: 2, now: 1721635200.5)
    #expect(s.engine == "mujoco")
    #expect(s.frame == 7)
    #expect(s.t == 1721635200.5)
    #expect(s.pose.count == m.ngeom)          // EVERY geom, not just visible
    #expect(s.pose[1].count == 7)             // x,y,z,qw,qx,qy,qz
    #expect(abs(s.pose[1][2] - 1.0) < 1e-4)   // cube geom at z=1
    #expect(s.level == 2)
}

@Test func stateEncodesHudMixedAndOmitsNilLevel() throws {
    let m = try MjModel.load(xml: boxScene)
    let d = MjData(m); mjForward(m, d)
    let s = buildState(m, d, frame: 0, hud: ["gate": .text("2/5"), "spd": .number(3.25)],
                       level: nil, now: 1.0)
    let obj = try JSONSerialization.jsonObject(
        with: JSONEncoder().encode(s)) as! [String: Any]
    #expect(obj["level"] == nil)                                  // omitted when nil
    let hud = obj["hud"] as! [String: Any]
    #expect(hud["gate"] as? String == "2/5")
    #expect((hud["spd"] as? Double).map { abs($0 - 3.25) < 1e-9 } == true)
    #expect(Set(obj.keys) == ["t", "frame", "engine", "pose", "contacts", "hud"])
}

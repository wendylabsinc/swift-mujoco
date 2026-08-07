// A tiny end-to-end smoke of the swift-mujoco binding: build a model, step real
// physics, read geom world poses, and detect the landing contact — all through
// the Swift API, no C in sight.
import MuJoCo
import WendyMuJoCo

func r(_ x: Double, _ places: Double = 1000) -> Double { (x * places).rounded() / places }

let xml = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.2 0.23 0.28 1"/>
    <body name="cube" pos="0 0 2">
      <freejoint/>
      <geom name="box" type="box" size="0.15 0.15 0.15" rgba="0.2 0.6 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"""

print("MuJoCo \(mujocoVersion())  (via swift-mujoco)")

let model = try MjModel.load(xml: xml)
let data = MjData(model)
guard let box = model.id(of: objGeom, name: "box") else { fatalError("no 'box' geom") }
print("model: \(model.ngeom) geoms, \(model.nbody) bodies, dt=\(model.timestep)s\n")

print("  step     t(s)   altitude(m)   contacts")
var landed = false
var recorder = WorldSimRecorder()
for step in 0...1200 {
    mjStep(model, data)
    recorder.record(model: model, data: data, title: "mujoco-demo: falling cube", frame: step)
    let z = data.geomXpos(box).z
    if step % 100 == 0 {
        print("  \(String(repeating: " ", count: max(0, 4 - String(step).count)))\(step)   \(r(data.time))       \(r(z))         \(data.ncon)")
    }
    if !landed && data.ncon > 0 {
        landed = true
        let peak = data.contacts().map(\.forceNormal).max() ?? 0
        print("  >> cube hit the floor at t=\(r(data.time))s  (peak normal force \(r(peak, 10))N)")
    }
}

print("\n  final resting altitude: \(r(data.geomXpos(box).z))m  (half-extent 0.15 → expect ~0.15)")

// Demonstrate full-physics state save/restore.
let saved = data.getFullState()
let tSaved = data.time
for _ in 0..<50 { mjStep(model, data) }
data.setFullState(saved)
print("  state save/restore: t \(r(tSaved))s → stepped → restored to \(r(data.time))s ✓")

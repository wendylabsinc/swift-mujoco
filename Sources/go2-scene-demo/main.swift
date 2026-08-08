// A minimal sample showing the scene DSL alongside a real robot: builds a
// floor-and-lighting world with `Scene { ... }`, then grafts the Unitree Go2
// (fetched via WendyMuJoCo's Menagerie helper) into it with `MjSpec.attach`.
import Foundation
import MuJoCo
import WendyMuJoCo

let world = Scene {
    Option(timestep: 0.002)
    Geom(name: "floor", type: .plane, size: [5, 5, 0.1], rgba: [0.3, 0.3, 0.3, 1])
    Light(pos: [0, 0, 3])
}

func resolveGo2Path() throws -> String {
    if let p = Menagerie.resolveModelPath("go2", searchDirs: Menagerie.vendorDirs, robot: true) {
        return p
    }
    print("Go2 model not found locally — fetching mujoco_menagerie (needs git + network)...")
    let cache = WorldSim.directory().appendingPathComponent("menagerie-cache")
    let repo = try Menagerie.fetch("go2", cacheDir: cache)
    guard let p = Menagerie.resolveModelPath("go2", searchDirs: [repo.path], robot: true) else {
        throw MjError("Go2 model still not found after fetching into \(repo.path)")
    }
    return p
}

let go2Path = try resolveGo2Path()
let go2 = try MjSpec(xmlPath: go2Path)

let spec = world.spec()
try spec.attach(go2, prefix: "go2_")
let model = try spec.compile()

print("Go2 scene compiled: \(model.nbody) bodies, \(model.njnt) joints, \(model.ngeom) geoms.")
for name in model.bodyNames where name.hasPrefix("go2_") {
    print("  body:", name)
}

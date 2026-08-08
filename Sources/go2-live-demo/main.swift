// A live, Hummingbird-served version of go2-scene-demo: builds the Go2 into a
// DSL world via MjSpec.attach, then steps physics continuously while
// embedding the HTTP router in the same process — mirrors mujoco-live-demo's
// falling-cube pattern, one process, no second one to manage.
import Foundation
import MuJoCo
import WendyMuJoCo
import WorldSimServerCore
import Hummingbird

let env = ProcessInfo.processInfo.environment
let port = env["PORT"].flatMap(Int.init) ?? 8789
let root = WorldSim.directory()

let app = Application(router: makeRouter(root: root),
                      configuration: .init(address: .hostname("127.0.0.1", port: port)))

print("go2-live-demo: http://127.0.0.1:\(port)  (slot: \(WorldSim.slot()), root: \(root.path))")
print("  GET /ctl/sim-running — Ctrl-C to stop")

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

try await withThrowingTaskGroup(of: Void.self) { group in
    group.addTask { try await app.runService() }
    group.addTask {
        // Declared here, not hoisted to top-level `let`s: `Scene`/`MjSpec`/
        // `MjModel` are all plain (non-`Sendable`) reference types, so
        // building and stepping the whole thing inside one task keeps it on
        // one isolation domain — matching mujoco-live-demo's own reasoning.
        let world = Scene {
            Option(timestep: 0.002)
            Geom(name: "floor", type: .plane, size: [5, 5, 0.1], rgba: [0.3, 0.3, 0.3, 1])
            Light(pos: [0, 0, 3])
        }
        let go2Path = try resolveGo2Path()
        let go2 = try MjSpec(xmlPath: go2Path)
        let spec = world.spec()
        try spec.attach(go2, prefix: "go2_")
        let model = try spec.compile()

        let data = MjData(model)
        let initialState = data.getFullState()   // t=0, Go2 at its authored pose
        var recorder = WorldSimRecorder()
        var frame = 0
        let stepNanos = UInt64(model.timestep * 1_000_000_000)
        // No actuation is applied, so the Go2 just settles under gravity like
        // a ragdoll — reset periodically so there's continuous motion to
        // watch instead of a static heap once it stops moving.
        let resetAfter = 6.0
        while !Task.isCancelled {
            mjStep(model, data)
            recorder.record(model: model, data: data,
                            title: "go2-live-demo: Go2 (looping)", frame: frame)
            frame += 1
            if data.time >= resetAfter { data.setFullState(initialState) }
            try await Task.sleep(nanoseconds: stepNanos)
        }
    }
    defer { group.cancelAll() }
    try await group.next()
}

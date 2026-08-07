// A single-process version of mujoco-demo + wendy-worldsim-server: runs the falling-cube
// physics loop live and looping, while embedding the HTTP router in the same process — one
// `swift run mujoco-live-demo` gives the desktop-native Sim tab something continuous to
// render, no second process to manage.
import Foundation
import MuJoCo
import WendyMuJoCo
import WorldSimServerCore
import Hummingbird

let env = ProcessInfo.processInfo.environment
let port = env["PORT"].flatMap(Int.init) ?? 8788
let root = WorldSim.directory()

let app = Application(router: makeRouter(root: root),
                      configuration: .init(address: .hostname("127.0.0.1", port: port)))

print("mujoco-live-demo: http://127.0.0.1:\(port)  (slot: \(WorldSim.slot()), root: \(root.path))")
print("  GET /ctl/sim-running — Ctrl-C to stop")

try await withThrowingTaskGroup(of: Void.self) { group in
    group.addTask { try await app.runService() }
    group.addTask {
        // Declared here, not hoisted to a top-level `let`: `Scene` is a plain
        // (non-`Sendable`) reference type, so building it inside the same
        // task that compiles and steps the resulting `MjModel` keeps both on
        // one isolation domain — matching the deliberate non-`Sendable`ness
        // of `MjModel`/`MjSpec` themselves (see `MjModel.swift`).
        let scene = Scene {
            Option(timestep: 0.002)
            Geom(name: "floor", type: .plane, size: [5, 5, 0.1], rgba: [0.2, 0.23, 0.28, 1])
            Body(name: "cube", pos: [0, 0, 2]) {
                FreeJoint()
                Geom(name: "box", type: .box, size: [0.15, 0.15, 0.15], rgba: [0.2, 0.6, 0.9, 1])
            }
        }
        let model = try scene.compile()
        let data = MjData(model)
        let initialState = data.getFullState()   // t=0, cube at rest height — replayed on every reset
        var recorder = WorldSimRecorder()
        var frame = 0
        let stepNanos = UInt64(model.timestep * 1_000_000_000)
        // Settles at ~2.4s (same physics as mujoco-demo's 1200-step run); pause a beat at rest,
        // then drop again so there's continuous motion to watch instead of a static end frame.
        let resetAfter = 4.0
        while !Task.isCancelled {
            mjStep(model, data)
            recorder.record(model: model, data: data,
                            title: "mujoco-live-demo: falling cube (looping)", frame: frame)
            frame += 1
            if data.time >= resetAfter { data.setFullState(initialState) }
            try await Task.sleep(nanoseconds: stepNanos)
        }
    }
    defer { group.cancelAll() }
    try await group.next()
}

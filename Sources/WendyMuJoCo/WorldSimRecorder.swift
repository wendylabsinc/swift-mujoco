import Foundation
import MuJoCo

/// Records a running MuJoCo sim to its `WorldSim` slot directory: the scene manifest once,
/// then one state frame per `record` call. Best-effort, matching `WorldSim.writeAtomic`'s
/// swallow-failures policy — a transient FS hiccup never crashes the sim loop.
public struct WorldSimRecorder {
    private let dir: URL
    private var sceneWritten = false

    public init(dir: URL = WorldSim.slotDirectory()) {
        self.dir = dir
    }

    public mutating func record(model: MjModel, data: MjData, title: String, frame: Int,
                                hud: [String: HUDValue] = [:], level: Int? = nil) {
        if !sceneWritten {
            let scene = buildScene(model, title: title)
            if let encoded = try? JSONEncoder().encode(scene) {
                WorldSim.writeAtomic(encoded, to: "scene.json", in: dir)
            }
            sceneWritten = true
        }
        let state = buildState(model, data, frame: frame, hud: hud, level: level, now: data.time)
        if let encoded = try? JSONEncoder().encode(state) {
            WorldSim.writeAtomic(encoded, to: "state.json", in: dir)
        }
    }
}

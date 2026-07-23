import Foundation
import MuJoCo
import CMuJoCo

public final class Handle {
    private let model: MjModel
    private let data: MjData
    private let dir: URL
    private var hudFields: [String: HUDValue]
    private var level: Int?
    private var frame = 0
    private var running = true

    private var resetSeen = 0
    private var stepSeen = 0
    private var pokeSeen = 0

    // Cache joint name/index -> (qposadr, dofadr) for pokes.
    private let jointQposAdr: [Int]   // by joint id
    private let jointDofAdr: [Int]

    public init(model: MjModel, data: MjData, title: String = "mujoco sim",
                hud: [String: HUDValue] = [:], dir: URL = WorldSim.directory()) {
        self.model = model
        self.data = data
        self.dir = dir
        self.hudFields = hud
        self.jointQposAdr = model.joints.map { $0.qposadr }
        self.jointDofAdr = model.joints.map { $0.dofadr }
        // Baseline counters so a stale flag from a previous sim doesn't fire on this one.
        let c = readControl(in: dir)
        self.resetSeen = c.reset
        self.stepSeen = c.step
        self.pokeSeen = c.poke
        writeScene(title: title)
    }

    public func isRunning() -> Bool { running }
    public func hud(_ fields: [String: HUDValue]) { hudFields = fields }
    public func setLevel(_ level: Int?) { self.level = level }
    public func close() { running = false }

    private func writeScene(title: String) {
        if let d = try? JSONEncoder().encode(buildScene(model, title: title)) {
            WorldSim.writeAtomic(d, to: "scene.json", in: dir)
        }
    }

    private func writeState() {
        frame += 1
        let s = buildState(model, data, frame: frame, hud: hudFields, level: level,
                           now: Date().timeIntervalSince1970)
        if let d = try? JSONEncoder().encode(s) {
            WorldSim.writeAtomic(d, to: "state.json", in: dir)
        }
    }

    private func resolveActuator(_ key: String) -> Int? {
        if let i = Int(key) { return (0..<model.nu).contains(i) ? i : nil }
        return model.id(of: objActuator, name: key)
    }
    private func resolveJoint(_ key: String) -> Int? {
        if let i = Int(key) { return (0..<model.njnt).contains(i) ? i : nil }
        return model.id(of: objJoint, name: key)
    }

    /// Reset when the counter advances. Returns true if it fired.
    private func applyReset(_ c: Control) -> Bool {
        guard c.reset != resetSeen else { return false }
        resetSeen = c.reset
        mjResetData(model, data)
        if model.nkey > 0 { mjResetDataKeyframe(model, data, 0) }
        return true
    }

    /// Persistent actuator setpoints, reapplied every frame so they hold.
    private func applyCtrl(_ c: Control) {
        for (k, v) in c.ctrl {
            if let aid = resolveActuator(k) { data.setCtrl(aid, v) }
        }
    }

    /// One-shot qpos/qvel poke when the counter advances. Returns true if it fired.
    private func applyPoke(_ c: Control) -> Bool {
        guard c.poke != pokeSeen else { return false }
        pokeSeen = c.poke
        for (k, v) in c.qpos {
            if let jid = resolveJoint(k) { data.ptr.pointee.qpos[jointQposAdr[jid]] = v }
        }
        for (k, v) in c.qvel {
            if let jid = resolveJoint(k) { data.ptr.pointee.qvel[jointDofAdr[jid]] = v }
        }
        mjForward(model, data)
        return true
    }

    public func sync() {
        var c = readControl(in: dir)
        _ = applyReset(c)
        applyCtrl(c)
        _ = applyPoke(c)
        writeState()
        // Pause: block after showing the frame until resumed or single-stepped.
        // Reset and pokes still take effect while paused.
        while c.paused && c.step == stepSeen && running {
            Thread.sleep(forTimeInterval: 0.04)
            c = readControl(in: dir)
            let fired = applyReset(c)
            let poked = applyPoke(c)
            if fired || poked { writeState() }
        }
        stepSeen = c.step
    }
}

/// Drop-in shaped like mujoco.viewer.launch_passive that renders in the Sim tab.
public func launchPassive(_ model: MjModel, _ data: MjData, title: String = "mujoco sim",
                          hud: [String: HUDValue] = [:]) -> Handle {
    Handle(model: model, data: data, title: title, hud: hud)
}

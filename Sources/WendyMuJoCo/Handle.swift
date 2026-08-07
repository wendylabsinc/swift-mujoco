import Foundation
import MuJoCo
import CMuJoCo

/// A flag that can be set from a thread other than the one running the sim loop.
///
/// `Handle.close()` needs to interrupt a `sync()` that is parked in the pause
/// loop, which by definition is not the caller's thread. Everything else in
/// `Handle` stays single-threaded; this is the one crossing point.
private final class AtomicFlag: @unchecked Sendable {
    private let lock = NSLock()
    private var value: Bool
    init(_ value: Bool) { self.value = value }
    var wrapped: Bool {
        get { lock.lock(); defer { lock.unlock() }; return value }
        set { lock.lock(); value = newValue; lock.unlock() }
    }
}

public final class Handle {
    /// Default state-publish rate. The sim loop can run at 200 Hz+, but the Sim
    /// tab renders at display rate and every publish costs a JSON encode plus an
    /// atomic write (temp file + rename). Publishing every step turned the sim
    /// loop into a file-creation benchmark — thousands of temp files a second,
    /// and real flash wear if the slot dir is not on tmpfs.
    public static let defaultStateHz: Double = 30
    /// Default control-poll rate. Control input is human-driven; polling it every
    /// step meant a `read` + full JSON decode per step for input that changes at
    /// most a few times a second.
    public static let defaultControlHz: Double = 20
    /// How long the pause loop sleeps between control polls.
    private static let pausePollInterval: Double = 0.04

    private let model: MjModel
    private let data: MjData
    private let dir: URL
    private var hudFields: [String: HUDValue]
    private var level: Int?
    private var frame = 0
    private let running = AtomicFlag(true)

    private var resetSeen = 0
    private var stepSeen = 0
    private var pokeSeen = 0

    /// Last successfully-read control. Reused verbatim when a read fails, so a
    /// transient FS hiccup is "no new information" rather than "every counter
    /// went back to zero" (which used to fire a spurious reset — see
    /// ``readControl(in:)``).
    private var lastControl = Control()

    private let stateInterval: Double
    private let controlInterval: Double
    private let now: @Sendable () -> Double
    private var lastStateAt: Double?
    private var lastControlAt: Double?

    // Cache joint name/index -> (qposadr, dofadr) for pokes.
    private let jointQposAdr: [Int]   // by joint id
    private let jointDofAdr: [Int]

    /// - Parameters:
    ///   - stateHz: how often to publish `state.json`. `nil` publishes on every
    ///     `sync()` (the pre-throttling behaviour), which is only sensible for a
    ///     sim loop that already runs at display rate.
    ///   - controlHz: how often to re-read `control.json`. `nil` reads every
    ///     `sync()`.
    ///   - now: monotonic-ish clock, injectable for tests.
    public init(model: MjModel, data: MjData, title: String = "mujoco sim",
                hud: [String: HUDValue] = [:], dir: URL = WorldSim.directory(),
                stateHz: Double? = Handle.defaultStateHz,
                controlHz: Double? = Handle.defaultControlHz,
                now: @escaping @Sendable () -> Double = { Date().timeIntervalSince1970 }) {
        precondition(stateHz == nil || stateHz! > 0, "stateHz must be positive or nil")
        precondition(controlHz == nil || controlHz! > 0, "controlHz must be positive or nil")
        self.model = model
        self.data = data
        self.dir = dir
        self.hudFields = hud
        self.stateInterval = stateHz.map { 1 / $0 } ?? 0
        self.controlInterval = controlHz.map { 1 / $0 } ?? 0
        self.now = now
        self.jointQposAdr = model.joints.map { $0.qposadr }
        self.jointDofAdr = model.joints.map { $0.dofadr }
        // Baseline counters so a stale flag from a previous sim doesn't fire on this one.
        if let c = readControl(in: dir) {
            self.lastControl = c
            self.resetSeen = c.reset
            self.stepSeen = c.step
            self.pokeSeen = c.poke
        }
        writeScene(title: title)
    }

    public func isRunning() -> Bool { running.wrapped }
    public func hud(_ fields: [String: HUDValue]) { hudFields = fields }
    public func setLevel(_ level: Int?) { self.level = level }

    /// Stop the sim. Safe to call from another thread — and it has to be, because
    /// the only time it is useful is while `sync()` is parked in the pause loop.
    public func close() { running.wrapped = false }

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

    /// Publish unless the last publish was less than `stateInterval` ago.
    /// `force` bypasses the throttle for events the UI must not miss (a reset or
    /// poke landing while paused, where no further frames are coming).
    private func publishState(force: Bool = false) {
        let t = now()
        if !force, stateInterval > 0, let last = lastStateAt, t - last < stateInterval { return }
        lastStateAt = t
        writeState()
    }

    /// Re-read control.json, subject to `controlInterval`. Returns the last known
    /// control when throttled or when the read fails.
    private func pollControl(force: Bool = false) -> Control {
        let t = now()
        if !force, controlInterval > 0, let last = lastControlAt, t - last < controlInterval {
            return lastControl
        }
        lastControlAt = t
        if let c = readControl(in: dir) { lastControl = c }
        return lastControl
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
    ///
    /// - Note: writes a single scalar at the joint's `qposadr`/`dofadr`, which is
    ///   the whole value for a hinge or slide but only the first component of a
    ///   ball (4) or free (7) joint.
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

    /// Apply pending control, publish a frame, and report whether the caller
    /// should park because the sim is paused.
    ///
    /// The shared body of ``sync()`` and ``syncAsync()`` — everything except *how*
    /// the wait is performed, which is the only thing that differs between the
    /// blocking and async forms.
    private func syncOnce() -> Bool {
        let c = pollControl()
        _ = applyReset(c)
        applyCtrl(c)
        _ = applyPoke(c)
        publishState()
        let shouldPark = c.paused && c.step == stepSeen && isRunning()
        if shouldPark {
            // Paused means no further frames, so make sure the UI is showing the
            // current one before we park.
            publishState(force: true)
        } else {
            stepSeen = c.step
        }
        return shouldPark
    }

    /// One iteration of the pause loop: re-read control, apply reset/poke, and
    /// report whether we are still paused.
    private func pauseTick() -> Bool {
        let c = pollControl(force: true)
        let fired = applyReset(c)
        let poked = applyPoke(c)
        if fired || poked { publishState(force: true) }
        guard c.paused && c.step == stepSeen && isRunning() else {
            stepSeen = c.step
            return false
        }
        return true
    }

    /// Publish a frame and, if paused, **block** until resumed or single-stepped.
    ///
    /// - Important: While paused this parks the calling thread with
    ///   `Thread.sleep`. That is fine on a thread you own, and wrong inside a
    ///   `Task`: it occupies a cooperative-pool thread for the whole pause, and
    ///   the pool has one thread per core. Driving a sim loop from a `Task` is the
    ///   natural thing to do, so prefer ``syncAsync()`` there — it suspends
    ///   instead, freeing the thread for other work.
    public func sync() {
        guard syncOnce() else { return }
        while pauseTick() {
            Thread.sleep(forTimeInterval: Self.pausePollInterval)
        }
    }

    /// Publish a frame and, if paused, **suspend** until resumed or single-stepped.
    ///
    /// The structured-concurrency form of ``sync()``: `Task.sleep` yields the
    /// cooperative-pool thread rather than holding it, so a paused sim costs no
    /// thread. It is also cancellation-aware — cancelling the surrounding task
    /// returns from the pause immediately, without needing ``close()``.
    ///
    /// The MuJoCo stepping itself is still synchronous and still has to happen on
    /// one isolation domain; this only changes how the *wait* is performed. Call it
    /// from wherever your `MjModel`/`MjData` live.
    public func syncAsync() async {
        guard syncOnce() else { return }
        while pauseTick() {
            do {
                try await Task.sleep(for: .seconds(Self.pausePollInterval))
            } catch {
                // Cancelled: stop waiting and let the caller unwind. Do not treat
                // this as "resumed" — leave stepSeen alone so the next sync still
                // sees the pause if the task is restarted.
                return
            }
        }
    }
}

/// Drop-in shaped like mujoco.viewer.launch_passive that renders in the Sim tab.
public func launchPassive(_ model: MjModel, _ data: MjData, title: String = "mujoco sim",
                          hud: [String: HUDValue] = [:]) -> Handle {
    Handle(model: model, data: data, title: title, hud: hud)
}

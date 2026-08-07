import Foundation

public struct Control: Decodable, Equatable {
    public var paused = false
    public var step = 0
    public var reset = 0
    public var poke = 0
    public var ctrl: [String: Double] = [:]
    public var qpos: [String: Double] = [:]
    public var qvel: [String: Double] = [:]

    public init() {}

    private enum CodingKeys: String, CodingKey {
        case paused, step, reset, poke, ctrl, qpos, qvel
    }

    /// Decodes leniently: a field of the wrong type falls back to its default
    /// rather than failing the whole document, because the renderer writing this
    /// file is a separate process on its own release cadence and one bad field
    /// should not freeze the sim.
    ///
    /// This is deliberately *not* the same as "the file was unreadable" — see
    /// ``readControl(in:)``, which distinguishes the two.
    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        func value<T: Decodable>(_ key: CodingKeys, default fallback: T) -> T {
            // Swift flattens `try?` over an already-Optional expression, so this
            // single nil covers both "key absent" and "present but wrong type".
            (try? c.decodeIfPresent(T.self, forKey: key)) ?? fallback
        }
        paused = value(.paused, default: false)
        step = value(.step, default: 0)
        reset = value(.reset, default: 0)
        poke = value(.poke, default: 0)
        ctrl = value(.ctrl, default: [:])
        qpos = value(.qpos, default: [:])
        qvel = value(.qvel, default: [:])
    }
}

/// Read control.json from `dir`.
///
/// Returns `nil` — not a default-valued `Control` — when the file is missing,
/// unreadable, or not valid JSON. That distinction matters: `Control()` has
/// `reset == step == poke == 0`, and the counters in `Handle` fire on *change*,
/// so handing back zeros for an unreadable file made a single transient read
/// failure look like "the counters went backwards" and spuriously fired a full
/// `mjResetData` (and blanked every ctrl setpoint for that frame). Callers must
/// treat `nil` as "no new information" and keep the last known control.
public func readControl(in dir: URL) -> Control? {
    guard let data = try? Data(contentsOf: dir.appendingPathComponent("control.json")),
          let c = try? JSONDecoder().decode(Control.self, from: data)
    else { return nil }
    return c
}

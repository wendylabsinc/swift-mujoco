import Foundation

public struct Control: Decodable {
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
    public init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        paused = (try? c.decodeIfPresent(Bool.self, forKey: .paused)) ?? nil ?? false
        step = (try? c.decodeIfPresent(Int.self, forKey: .step)) ?? nil ?? 0
        reset = (try? c.decodeIfPresent(Int.self, forKey: .reset)) ?? nil ?? 0
        poke = (try? c.decodeIfPresent(Int.self, forKey: .poke)) ?? nil ?? 0
        ctrl = (try? c.decodeIfPresent([String: Double].self, forKey: .ctrl)) ?? nil ?? [:]
        qpos = (try? c.decodeIfPresent([String: Double].self, forKey: .qpos)) ?? nil ?? [:]
        qvel = (try? c.decodeIfPresent([String: Double].self, forKey: .qvel)) ?? nil ?? [:]
    }
}

/// Read control.json from `dir`; any missing file / parse error yields all-defaults.
public func readControl(in dir: URL) -> Control {
    guard let data = try? Data(contentsOf: dir.appendingPathComponent("control.json")),
          let c = try? JSONDecoder().decode(Control.self, from: data)
    else { return Control() }
    return c
}

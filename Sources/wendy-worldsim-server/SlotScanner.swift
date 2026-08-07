import Foundation

/// One slot directory with a live (recently-heartbeating) state.json.
struct LiveSlot: Equatable {
    let name: String
    let stateModified: Date
}

/// Slots under `root` whose `state.json` was modified within `heartbeatSeconds` of `now`,
/// newest first. A slot directory with no `state.json`, or a stale one, is not "live" — this
/// is how `/ctl/sim-running` tells a running sim from an abandoned/dead one.
func liveSlots(root: URL, heartbeatSeconds: TimeInterval, now: Date,
              fileManager: FileManager = .default) -> [LiveSlot] {
    guard let entries = try? fileManager.contentsOfDirectory(
        at: root, includingPropertiesForKeys: [.isDirectoryKey], options: [.skipsHiddenFiles])
    else { return [] }

    var slots: [LiveSlot] = []
    for entry in entries {
        guard (try? entry.resourceValues(forKeys: [.isDirectoryKey]))?.isDirectory == true else { continue }
        let stateFile = entry.appendingPathComponent("state.json")
        guard let attrs = try? fileManager.attributesOfItem(atPath: stateFile.path),
              let modified = attrs[.modificationDate] as? Date,
              now.timeIntervalSince(modified) <= heartbeatSeconds
        else { continue }
        slots.append(LiveSlot(name: entry.lastPathComponent, stateModified: modified))
    }
    return slots.sorted { $0.stateModified > $1.stateModified }
}

/// Caches each slot's `scene.json` title after the first successful read, so repeated
/// `/ctl/sim-running` polls (every 1.5s from the Sim tab) don't re-parse scene.json — which
/// can hold large mesh vertex arrays — on every call.
actor SceneTitleCache {
    private var titles: [String: String] = [:]

    func title(forSlot slot: String, in slotDir: URL, fileManager: FileManager = .default) -> String? {
        if let cached = titles[slot] { return cached }
        guard let data = try? Data(contentsOf: slotDir.appendingPathComponent("scene.json")),
              let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let title = obj["title"] as? String
        else { return nil }
        titles[slot] = title
        return title
    }
}

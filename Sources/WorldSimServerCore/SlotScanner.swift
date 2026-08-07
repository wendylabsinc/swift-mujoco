import Foundation

/// One slot directory with a live (recently-heartbeating) state.json.
struct LiveSlot: Equatable {
    let name: String
    let stateModified: Date
}

/// Slots under `root` whose `state.json` was modified within `heartbeatSeconds` of `now`,
/// sorted by name so the `running` list order is stable across polls regardless of write
/// timing. A slot directory with no `state.json`, or a stale one, is not "live" — this is
/// how `/ctl/sim-running` tells a running sim from an abandoned/dead one.
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
    return slots.sorted { $0.name < $1.name }
}

/// Caches each slot's `scene.json` title, keyed by the file's mtime, so repeated
/// `/ctl/sim-running` polls (every 1.5s from the Sim tab) don't re-parse scene.json — which
/// can hold large mesh vertex arrays — unless the file has actually changed since the last
/// read. This also means a slot name reused for a different sim (new scene.json, new mtime)
/// picks up the new title instead of serving the previous sim's title forever.
actor SceneTitleCache {
    private var entries: [String: (title: String, mtime: Date)] = [:]

    func title(forSlot slot: String, in slotDir: URL, fileManager: FileManager = .default) -> String? {
        let sceneFile = slotDir.appendingPathComponent("scene.json")
        let currentMTime = (try? fileManager.attributesOfItem(atPath: sceneFile.path))?[.modificationDate] as? Date
        if let currentMTime, let cached = entries[slot], cached.mtime == currentMTime {
            return cached.title   // unchanged since last read — skip the reparse
        }
        guard let data = try? Data(contentsOf: sceneFile),
              let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any],
              let title = obj["title"] as? String
        else {
            return entries[slot]?.title   // stat/read failed (transient hiccup, or file briefly gone) — serve last known good value
        }
        if let currentMTime { entries[slot] = (title, currentMTime) }
        return title
    }
}

/// Sticky focus tracker for `/ctl/sim-running`: keeps reporting the same slot while it's
/// still live, only re-picking (alphabetically first, matching `liveSlots`' stable ordering)
/// once it goes stale. This is what keeps a client's "active sim" from bouncing every ~1.5s
/// poll when multiple sims are running concurrently.
actor FocusTracker {
    private var current: String?

    func resolve(liveNames: [String]) -> String? {
        if let current, liveNames.contains(current) { return current }
        let next = liveNames.sorted().first
        current = next
        return next
    }
}

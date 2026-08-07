# Local swift-mujoco → Sim tab bridge (swift-mujoco side) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let any swift-mujoco-driven simulation stream its scene/state to disk and serve it over a small local HTTP shim, so the wendy-sandbox native Sim tab can render it without a Docker session container.

**Architecture:** `WendyMuJoCo` gains a per-process "slot" directory convention and a `WorldSimRecorder` that writes `scene.json`/`state.json` there every step (reusing the existing `buildScene`/`buildState`/`WorldSim.writeAtomic` primitives). A new `wendy-worldsim-server` executable (Hummingbird) discovers live slots by scanning that directory and serves the exact `/ctl/sim-running`, `/ctl/sim-list`, `/simslot/<slot>/{scene,state}.json` shape the Sim tab's `LiveSimClient` already speaks. `mujoco-demo` is wired to the recorder as the first live consumer.

**Tech Stack:** Swift 6.1, Swift Testing (`import Testing`, `@Test`, `#expect`), Hummingbird 2 (HTTP server), [swift-json-schema](https://github.com/wendylabsinc/swift-json-schema) (schema-driven `Codable` model generation for the small JSON control responses).

**Full design:** `wendy-sandbox/docs/superpowers/specs/2026-08-06-local-swift-mujoco-sim-bridge-design.md` (this repo's pointer: `docs/superpowers/specs/2026-08-06-local-swift-mujoco-sim-bridge-design.md`)

## Global Constraints

- Swift tools version: `6.1` (matches existing `Package.swift`).
- Package-wide platform floor is `.macOS(.v14)` — this only gates macOS builds; it does not block Linux. `wendy-worldsim-server` must build and pass tests on both macOS and Linux (unlike `MujocoRLDemo`, do NOT gate it behind `#if os(macOS)`).
- Tests use the **Swift Testing** framework (`import Testing`, `@Test func`, `#expect`) — matches every existing test file under `Tests/`, not XCTest.
- `WorldSim.writeAtomic` is best-effort (swallows failures) — new code built on top of it (`WorldSimRecorder`) must preserve that: never `try!`/crash on a write failure.
- Do not modify the `#if os(macOS)` block that gates `MujocoRLDemo`/`mlx-swift`.

---

### Task 1: `WorldSim` slot-directory convention

**Files:**
- Modify: `Sources/WendyMuJoCo/WorldSim.swift`
- Test: Modify `Tests/WendyMuJoCoTests/WorldSimTests.swift`

**Interfaces:**
- Produces: `WorldSim.slot(_ env: [String: String] = ProcessInfo.processInfo.environment) -> String`, `WorldSim.slotDirectory() -> URL`. Later tasks (`WorldSimRecorder`, `wendy-worldsim-server`) use `WorldSim.slotDirectory()` as the default per-process write location and `WorldSim.directory()` (existing) as the shim server's scan root.

- [ ] **Step 1: Write the failing tests**

Add to `Tests/WendyMuJoCoTests/WorldSimTests.swift` (append after the existing `roundingMatchesPlaces` test):

```swift
@Test func slotDefaultsToDefault() {
    #expect(WorldSim.slot([:]) == "default")
}

@Test func slotHonorsEnvOverride() {
    #expect(WorldSim.slot(["WENDY_WORLDSIM_SLOT": "cartpole"]) == "cartpole")
}

@Test func slotTreatsEmptyOverrideAsUnset() {
    #expect(WorldSim.slot(["WENDY_WORLDSIM_SLOT": ""]) == "default")
}

@Test func slotDirectoryNestsSlotUnderDirectory() {
    let expected = WorldSim.directory().appendingPathComponent(WorldSim.slot(), isDirectory: true)
    #expect(WorldSim.slotDirectory().path == expected.path)
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig swift test --filter WorldSimTests`
Expected: FAIL — `slot`/`slotDirectory` are not members of `WorldSim`.

- [ ] **Step 3: Implement**

In `Sources/WendyMuJoCo/WorldSim.swift`, add to the `WorldSim` enum (after `directory()`):

```swift
    /// The slot name for this process: $WENDY_WORLDSIM_SLOT, or "default". Multiple
    /// swift-mujoco processes can run concurrently, each in its own slot, so
    /// wendy-worldsim-server can discover and serve them independently.
    public static func slot(_ env: [String: String] = ProcessInfo.processInfo.environment) -> String {
        let v = env["WENDY_WORLDSIM_SLOT"]
        return (v?.isEmpty ?? true) ? "default" : v!
    }

    /// This process's slot directory: `directory()/slot()`. Every WorldSim write for a
    /// single running sim belongs under one slot dir.
    public static func slotDirectory() -> URL {
        directory().appendingPathComponent(slot(), isDirectory: true)
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig swift test --filter WorldSimTests`
Expected: PASS (7 tests: 3 existing + 4 new).

- [ ] **Step 5: Commit**

```bash
git add Sources/WendyMuJoCo/WorldSim.swift Tests/WendyMuJoCoTests/WorldSimTests.swift
git commit -m "feat: add WorldSim slot-directory convention"
```

---

### Task 2: `WorldSimRecorder`

**Files:**
- Create: `Sources/WendyMuJoCo/WorldSimRecorder.swift`
- Test: Create `Tests/WendyMuJoCoTests/WorldSimRecorderTests.swift`

**Interfaces:**
- Consumes: `WorldSim.slotDirectory()`, `WorldSim.writeAtomic(_:to:in:)`, `WorldSim.path(_:in:)` (Task 1 / existing), `buildScene(_:title:) -> SceneManifest`, `buildState(_:_:frame:hud:level:now:) -> StateFrame`, `HUDValue` (existing, `SceneManifest.swift`/`StateFrame.swift`).
- Produces: `struct WorldSimRecorder { init(dir: URL = WorldSim.slotDirectory()); mutating func record(model: MjModel, data: MjData, title: String, frame: Int, hud: [String: HUDValue] = [:], level: Int? = nil) }`. Task 3 (`mujoco-demo`) constructs and calls this every step.

- [ ] **Step 1: Write the failing test**

Create `Tests/WendyMuJoCoTests/WorldSimRecorderTests.swift`:

```swift
import Testing
import Foundation
import MuJoCo
@testable import WendyMuJoCo

private let boxScene = """
<mujoco><worldbody>
  <geom name="floor" type="plane" size="5 5 0.1"/>
  <body name="cube" pos="0 0 1"><freejoint/>
    <geom name="box" type="box" size="0.1 0.1 0.1"/>
  </body>
</worldbody></mujoco>
"""

@Test func recorderWritesSceneOnceAndStateOnEveryCall() throws {
    let dir = FileManager.default.temporaryDirectory.appendingPathComponent("wsr-\(UUID().uuidString)")
    defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: boxScene)
    let d = MjData(m)
    mjForward(m, d)
    var recorder = WorldSimRecorder(dir: dir)

    recorder.record(model: m, data: d, title: "first", frame: 0)
    recorder.record(model: m, data: d, title: "second", frame: 1)   // title change ignored: scene already written

    let scene = try Data(contentsOf: WorldSim.path("scene.json", in: dir))
    let sceneObj = try JSONSerialization.jsonObject(with: scene) as! [String: Any]
    #expect(sceneObj["title"] as? String == "first")   // written once, on the first call

    let state = try Data(contentsOf: WorldSim.path("state.json", in: dir))
    let stateObj = try JSONSerialization.jsonObject(with: state) as! [String: Any]
    #expect(stateObj["frame"] as? Int == 1)            // state.json reflects the latest record()
}

@Test func recorderPassesHudAndLevelThrough() throws {
    let dir = FileManager.default.temporaryDirectory.appendingPathComponent("wsr-\(UUID().uuidString)")
    defer { try? FileManager.default.removeItem(at: dir) }
    let m = try MjModel.load(xml: boxScene)
    let d = MjData(m)
    mjForward(m, d)
    var recorder = WorldSimRecorder(dir: dir)

    recorder.record(model: m, data: d, title: "t", frame: 3, hud: ["gate": .text("2/5")], level: 2)

    let state = try Data(contentsOf: WorldSim.path("state.json", in: dir))
    let obj = try JSONSerialization.jsonObject(with: state) as! [String: Any]
    #expect((obj["hud"] as! [String: Any])["gate"] as? String == "2/5")
    #expect(obj["level"] as? Int == 2)
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig swift test --filter WorldSimRecorderTests`
Expected: FAIL — `WorldSimRecorder` does not exist.

- [ ] **Step 3: Implement**

Create `Sources/WendyMuJoCo/WorldSimRecorder.swift`:

```swift
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig swift test --filter WorldSimRecorderTests`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add Sources/WendyMuJoCo/WorldSimRecorder.swift Tests/WendyMuJoCoTests/WorldSimRecorderTests.swift
git commit -m "feat: add WorldSimRecorder to stream scene/state to a slot directory"
```

---

### Task 3: Wire the recorder into `mujoco-demo`

**Files:**
- Modify: `Package.swift` (`MujocoDemo` target dependencies)
- Modify: `Sources/mujoco-demo/main.swift`

**Interfaces:**
- Consumes: `WorldSimRecorder` (Task 2).
- Produces: nothing consumed by later tasks — this is a leaf/manual-smoke task. `mujoco-demo` has no test target today (matches existing repo layout — `MujocoDemo` is the only executable target with no corresponding `*Tests` target), so this task's verification is a manual run, not `swift test`.

- [ ] **Step 1: Add `WendyMuJoCo` as a dependency of `MujocoDemo`**

In `Package.swift`, change:

```swift
    .executableTarget(name: "MujocoDemo", dependencies: ["MuJoCo"], path: "Sources/mujoco-demo"),
```

to:

```swift
    .executableTarget(name: "MujocoDemo", dependencies: ["MuJoCo", "WendyMuJoCo"], path: "Sources/mujoco-demo"),
```

- [ ] **Step 2: Wire the recorder into the step loop**

In `Sources/mujoco-demo/main.swift`, add the import at the top:

```swift
import MuJoCo
import WendyMuJoCo
```

Then wire a recorder into the existing step loop — change:

```swift
print("  step     t(s)   altitude(m)   contacts")
var landed = false
for step in 0...1200 {
    mjStep(model, data)
    let z = data.geomXpos(box).z
```

to:

```swift
print("  step     t(s)   altitude(m)   contacts")
var landed = false
var recorder = WorldSimRecorder()
for step in 0...1200 {
    mjStep(model, data)
    recorder.record(model: model, data: data, title: "mujoco-demo: falling cube", frame: step)
    let z = data.geomXpos(box).z
```

- [ ] **Step 3: Build and manually verify**

Run:

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift build
WENDY_WORLDSIM_SLOT=demo .build/debug/mujoco-demo
cat /tmp/wendy-worldsim/demo/scene.json | python3 -m json.tool | head -20
cat /tmp/wendy-worldsim/demo/state.json | python3 -m json.tool
```

Expected: `mujoco-demo` prints its usual physics trace; `scene.json` has `title`, `up`, `engine`, `geoms`; `state.json` has `frame` (near 1200, the loop's last written frame), `engine: "mujoco"`, `pose`, `contacts`.

- [ ] **Step 4: Commit**

```bash
git add Package.swift Sources/mujoco-demo/main.swift
git commit -m "feat: stream mujoco-demo's run to its WorldSim slot"
```

---

### Task 4: `wendy-worldsim-server` scaffold — dependencies, target, and wire schemas

**Files:**
- Modify: `Package.swift` (new dependencies, `WendyWorldSimServer` executable target, `WendyWorldSimServerTests` test target, `wendy-worldsim-server` product)
- Create: `Sources/wendy-worldsim-server/SimRunningResponse.schema.json`
- Create: `Sources/wendy-worldsim-server/SimListResponse.schema.json`
- Create: `Sources/wendy-worldsim-server/SimControlResponse.schema.json`
- Create: `Sources/wendy-worldsim-server/WendyWorldSimServer.swift`
- Test: Create `Tests/WendyWorldSimServerTests/GeneratedTypesTests.swift`

**Interfaces:**
- Produces (from the `swift-json-schema` codegen, consumed by Task 5 & 6): `struct RunningSim: Codable, Hashable { let slot: String; let file: String; var title: String? }`, `struct SimRunningResponse: Codable, Hashable { let running: [RunningSim]; var focus: String? }`, `struct SimListResponse: Codable, Hashable { let sims: [String]; var current: String? }`, `struct SimControl: Codable, Hashable { var paused: Bool?; var step: Int?; var reset: Int? }`, `struct SimControlResponse: Codable, Hashable { let ok: Bool; var control: SimControl? }`.

- [ ] **Step 1: Add dependencies and targets to `Package.swift`**

Add the two new dependencies right after `var dependencies: [Package.Dependency] = []` (before the `#if os(macOS)` block, so they're unconditional/cross-platform):

```swift
var dependencies: [Package.Dependency] = [
    .package(url: "https://github.com/hummingbird-project/hummingbird.git", from: "2.0.0"),
    .package(url: "https://github.com/wendylabsinc/swift-json-schema.git", from: "0.1.0"),
    // Referenced directly (ByteBuffer in Routes.swift, Data(buffer:) in RoutesTests.swift) —
    // SPM requires a package to be listed here to use its products, even though Hummingbird
    // already depends on it transitively.
    .package(url: "https://github.com/apple/swift-nio.git", from: "2.60.0"),
]
```

Add two entries to the `targets` array (anywhere after the `WendyMuJoCo`/`WendyMuJoCoTests` pair):

```swift
    .executableTarget(
        name: "WendyWorldSimServer",
        dependencies: [
            "WendyMuJoCo",
            .product(name: "Hummingbird", package: "hummingbird"),
            .product(name: "NIOCore", package: "swift-nio"),
        ],
        path: "Sources/wendy-worldsim-server",
        plugins: [.plugin(name: "JSONSchemaPlugin", package: "swift-json-schema")]
    ),
    .testTarget(
        name: "WendyWorldSimServerTests",
        dependencies: [
            "WendyWorldSimServer",
            .product(name: "HummingbirdTesting", package: "hummingbird"),
            .product(name: "NIOCore", package: "swift-nio"),
            .product(name: "NIOFoundationCompat", package: "swift-nio"),
        ]
    ),
```

Add to `products`:

```swift
    .executable(name: "wendy-worldsim-server", targets: ["WendyWorldSimServer"]),
```

- [ ] **Step 2: Write the failing test**

Create `Tests/WendyWorldSimServerTests/GeneratedTypesTests.swift`:

```swift
import Testing
import Foundation
@testable import WendyWorldSimServer

@Test func simRunningResponseRoundTripsThroughJSON() throws {
    let value = SimRunningResponse(running: [RunningSim(slot: "default", file: "default", title: "Falling cube")],
                                   focus: "default")
    let data = try JSONEncoder().encode(value)
    let decoded = try JSONDecoder().decode(SimRunningResponse.self, from: data)
    #expect(decoded == value)
    let obj = try JSONSerialization.jsonObject(with: data) as! [String: Any]
    #expect(Set(obj.keys) == ["running", "focus"])
}

@Test func simListResponseRoundTripsThroughJSON() throws {
    let value = SimListResponse(sims: [], current: nil)
    let decoded = try JSONDecoder().decode(SimListResponse.self, from: JSONEncoder().encode(value))
    #expect(decoded == value)
}

@Test func simControlResponseRoundTripsThroughJSON() throws {
    let value = SimControlResponse(ok: true, control: SimControl(paused: true, step: nil, reset: nil))
    let decoded = try JSONDecoder().decode(SimControlResponse.self, from: JSONEncoder().encode(value))
    #expect(decoded == value)
}
```

- [ ] **Step 3: Run test to verify it fails**

Run: `swift test --filter WendyWorldSimServerTests`
Expected: FAIL to build — `WendyWorldSimServer` module / `SimRunningResponse` etc. don't exist yet (no schema files, no source file in the target).

- [ ] **Step 4: Add the schema files**

Create `Sources/wendy-worldsim-server/SimRunningResponse.schema.json`:

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "SimRunningResponse",
  "type": "object",
  "properties": {
    "running": {
      "type": "array",
      "items": { "$ref": "#/$defs/RunningSim" }
    },
    "focus": { "type": ["string", "null"] }
  },
  "required": ["running"],
  "$defs": {
    "RunningSim": {
      "type": "object",
      "properties": {
        "slot": { "type": "string" },
        "file": { "type": "string" },
        "title": { "type": ["string", "null"] }
      },
      "required": ["slot", "file"]
    }
  }
}
```

Create `Sources/wendy-worldsim-server/SimListResponse.schema.json`:

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "SimListResponse",
  "type": "object",
  "properties": {
    "sims": { "type": "array", "items": { "type": "string" } },
    "current": { "type": ["string", "null"] }
  },
  "required": ["sims"]
}
```

Create `Sources/wendy-worldsim-server/SimControlResponse.schema.json`:

```json
{
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "title": "SimControlResponse",
  "type": "object",
  "properties": {
    "ok": { "type": "boolean" },
    "control": { "$ref": "#/$defs/SimControl" }
  },
  "required": ["ok"],
  "$defs": {
    "SimControl": {
      "type": "object",
      "properties": {
        "paused": { "type": ["boolean", "null"] },
        "step": { "type": ["integer", "null"] },
        "reset": { "type": ["integer", "null"] }
      }
    }
  }
}
```

Create `Sources/wendy-worldsim-server/WendyWorldSimServer.swift` (a minimal, compiling entry point — the real router is wired in Task 6):

```swift
@main
struct WendyWorldSimServer {
    static func main() async throws {
        print("wendy-worldsim-server: routes not yet wired (see Task 6)")
    }
}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `swift test --filter WendyWorldSimServerTests`
Expected: PASS (3 tests) — the plugin generated `RunningSim`, `SimRunningResponse`, `SimListResponse`, `SimControl`, `SimControlResponse` from the schema files.

- [ ] **Step 6: Commit**

```bash
git add Package.swift Sources/wendy-worldsim-server Tests/WendyWorldSimServerTests
git commit -m "feat: scaffold wendy-worldsim-server with schema-generated wire types"
```

---

### Task 5: Slot discovery and scene-title cache

**Files:**
- Create: `Sources/wendy-worldsim-server/SlotScanner.swift`
- Test: Create `Tests/WendyWorldSimServerTests/SlotScannerTests.swift`

**Interfaces:**
- Produces: `struct LiveSlot: Equatable { let name: String; let stateModified: Date }`, `func liveSlots(root: URL, heartbeatSeconds: TimeInterval, now: Date, fileManager: FileManager = .default) -> [LiveSlot]`, `actor SceneTitleCache { func title(forSlot: String, in: URL, fileManager: FileManager = .default) -> String? }`. Task 6's routes call both.

- [ ] **Step 1: Write the failing tests**

Create `Tests/WendyWorldSimServerTests/SlotScannerTests.swift`:

```swift
import Testing
import Foundation
@testable import WendyWorldSimServer

private func makeSlot(_ root: URL, name: String, stateAge: TimeInterval,
                      fileManager: FileManager = .default) throws {
    let dir = root.appendingPathComponent(name, isDirectory: true)
    try fileManager.createDirectory(at: dir, withIntermediateDirectories: true)
    let stateFile = dir.appendingPathComponent("state.json")
    try Data("{}".utf8).write(to: stateFile)
    try fileManager.setAttributes([.modificationDate: Date().addingTimeInterval(-stateAge)],
                                  ofItemAtPath: stateFile.path)
}

private func withTempRoot(_ body: (URL) throws -> Void) throws {
    let root = FileManager.default.temporaryDirectory.appendingPathComponent("slots-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: root, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: root) }
    try body(root)
}

@Test func liveSlotsExcludesStaleHeartbeats() throws {
    try withTempRoot { root in
        try makeSlot(root, name: "fresh", stateAge: 1)
        try makeSlot(root, name: "stale", stateAge: 30)
        let slots = liveSlots(root: root, heartbeatSeconds: 5, now: Date())
        #expect(slots.map(\.name) == ["fresh"])
    }
}

@Test func liveSlotsOrdersNewestFirst() throws {
    try withTempRoot { root in
        try makeSlot(root, name: "older", stateAge: 3)
        try makeSlot(root, name: "newer", stateAge: 1)
        let slots = liveSlots(root: root, heartbeatSeconds: 5, now: Date())
        #expect(slots.map(\.name) == ["newer", "older"])
    }
}

@Test func liveSlotsIgnoresDirectoryWithNoStateFile() throws {
    try withTempRoot { root in
        try FileManager.default.createDirectory(at: root.appendingPathComponent("empty"),
                                                 withIntermediateDirectories: true)
        #expect(liveSlots(root: root, heartbeatSeconds: 5, now: Date()).isEmpty)
    }
}

@Test func sceneTitleCacheReadsOnceAndCaches() async throws {
    try withTempRoot { root in
        try Data(#"{"title":"Falling cube"}"#.utf8).write(to: root.appendingPathComponent("scene.json"))
    }
    // withTempRoot removed `root` on exit above; recreate a persistent dir for the actor test.
    let dir = FileManager.default.temporaryDirectory.appendingPathComponent("title-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: dir) }
    try Data(#"{"title":"Falling cube"}"#.utf8).write(to: dir.appendingPathComponent("scene.json"))

    let cache = SceneTitleCache()
    let first = await cache.title(forSlot: "s", in: dir)
    #expect(first == "Falling cube")

    try FileManager.default.removeItem(at: dir.appendingPathComponent("scene.json"))   // prove it's cached
    let second = await cache.title(forSlot: "s", in: dir)
    #expect(second == "Falling cube")
}

@Test func sceneTitleCacheReturnsNilWhenSceneMissing() async {
    let dir = FileManager.default.temporaryDirectory.appendingPathComponent("title-\(UUID().uuidString)")
    let cache = SceneTitleCache()
    #expect(await cache.title(forSlot: "s", in: dir) == nil)
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `swift test --filter SlotScannerTests`
Expected: FAIL — `liveSlots`/`LiveSlot`/`SceneTitleCache` don't exist.

- [ ] **Step 3: Implement**

Create `Sources/wendy-worldsim-server/SlotScanner.swift`:

```swift
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `swift test --filter SlotScannerTests`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add Sources/wendy-worldsim-server/SlotScanner.swift Tests/WendyWorldSimServerTests/SlotScannerTests.swift
git commit -m "feat: add slot discovery and scene-title caching to wendy-worldsim-server"
```

---

### Task 6: Routes and server wiring

**Files:**
- Create: `Sources/wendy-worldsim-server/ResponseCodableConformances.swift`
- Create: `Sources/wendy-worldsim-server/Routes.swift`
- Modify: `Sources/wendy-worldsim-server/WendyWorldSimServer.swift`
- Test: Create `Tests/WendyWorldSimServerTests/RoutesTests.swift`

**Interfaces:**
- Consumes: `liveSlots`, `LiveSlot`, `SceneTitleCache` (Task 5); `SimRunningResponse`, `RunningSim`, `SimListResponse`, `SimControlResponse`, `SimControl` (Task 4); `WorldSim.directory()` (Task 1 / existing, from `WendyMuJoCo`).
- Produces: `func makeRouter(root: URL, heartbeatSeconds: TimeInterval = 5) -> Router<BasicRequestContext>` — the full HTTP surface. Nothing downstream in this repo depends on it (`main()` is the only caller); wendy-sandbox's plan does not import this module, it only talks to it over HTTP.

- [ ] **Step 1: Write the failing tests**

Create `Tests/WendyWorldSimServerTests/RoutesTests.swift`:

```swift
import Testing
import Foundation
import Hummingbird
import HummingbirdTesting
import NIOFoundationCompat
@testable import WendyWorldSimServer

private func withTempRoot(_ body: (URL) async throws -> Void) async throws {
    let root = FileManager.default.temporaryDirectory.appendingPathComponent("routes-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: root, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: root) }
    try await body(root)
}

@Test func simRunningListsOnlyLiveSlots() async throws {
    try await withTempRoot { root in
        let dir = root.appendingPathComponent("cube", isDirectory: true)
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        try Data(#"{"title":"Falling cube"}"#.utf8).write(to: dir.appendingPathComponent("scene.json"))
        try Data("{}".utf8).write(to: dir.appendingPathComponent("state.json"))

        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-running", method: .get) { response in
                let body = try JSONDecoder().decode(SimRunningResponse.self, from: Data(buffer: response.body))
                #expect(body.running.map(\.slot) == ["cube"])
                #expect(body.running[0].title == "Falling cube")
                #expect(body.focus == "cube")
            }
        }
    }
}

@Test func simRunningIsEmptyWithNoSlots() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-running", method: .get) { response in
                let body = try JSONDecoder().decode(SimRunningResponse.self, from: Data(buffer: response.body))
                #expect(body.running.isEmpty)
                #expect(body.focus == nil)
            }
        }
    }
}

@Test func simListIsAlwaysEmpty() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-list", method: .get) { response in
                let body = try JSONDecoder().decode(SimListResponse.self, from: Data(buffer: response.body))
                #expect(body.sims.isEmpty)
                #expect(body.current == nil)
            }
        }
    }
}

@Test func sceneJsonStreamsFileBytes() async throws {
    try await withTempRoot { root in
        let dir = root.appendingPathComponent("cube", isDirectory: true)
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        try Data(#"{"title":"Falling cube"}"#.utf8).write(to: dir.appendingPathComponent("scene.json"))

        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/simslot/cube/scene.json", method: .get) { response in
                #expect(response.status == .ok)
                #expect(Data(buffer: response.body) == Data(#"{"title":"Falling cube"}"#.utf8))
            }
        }
    }
}

@Test func sceneJsonReturns404ForMissingSlot() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/simslot/nope/scene.json", method: .get) { response in
                #expect(response.status == .notFound)
            }
        }
    }
}

@Test func simOpenIsANoOpThatReturns501() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-open", method: .post) { response in
                #expect(response.status.code == 501)
            }
        }
    }
}

@Test func simCmdIsANoOpThatReturnsOk() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-cmd", method: .post) { response in
                let body = try JSONDecoder().decode(SimControlResponse.self, from: Data(buffer: response.body))
                #expect(body.ok == true)
            }
        }
    }
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `swift test --filter RoutesTests`
Expected: FAIL — `makeRouter` doesn't exist.

- [ ] **Step 3: Implement the response conformances**

Create `Sources/wendy-worldsim-server/ResponseCodableConformances.swift`:

```swift
import Hummingbird

extension SimRunningResponse: ResponseCodable {}
extension SimListResponse: ResponseCodable {}
extension SimControlResponse: ResponseCodable {}
```

- [ ] **Step 4: Implement the routes**

Create `Sources/wendy-worldsim-server/Routes.swift`:

```swift
import Foundation
import Hummingbird
import NIOCore

/// Builds the wendy-worldsim-server router: the same `/ctl/sim-*` and `/simslot/*.json`
/// shape the desktop-native Sim tab's `LiveSimClient` already speaks (see
/// wendy-sandbox/docs/superpowers/specs/2026-08-06-local-swift-mujoco-sim-bridge-design.md),
/// backed by `root`'s slot directories instead of a session container.
func makeRouter(root: URL, heartbeatSeconds: TimeInterval = 5) -> Router<BasicRequestContext> {
    let router = Router()
    let titleCache = SceneTitleCache()

    router.get("/ctl/sim-running") { _, _ -> SimRunningResponse in
        let slots = liveSlots(root: root, heartbeatSeconds: heartbeatSeconds, now: Date())
        var running: [RunningSim] = []
        for slot in slots {
            let title = await titleCache.title(forSlot: slot.name, in: root.appendingPathComponent(slot.name))
            running.append(RunningSim(slot: slot.name, file: slot.name, title: title))
        }
        return SimRunningResponse(running: running, focus: slots.first?.name)
    }

    router.get("/ctl/sim-list") { _, _ -> SimListResponse in
        SimListResponse(sims: [], current: nil)
    }

    router.get("/simslot/{slot}/scene.json") { _, context in
        try slotFileResponse(root: root, context: context, fileName: "scene.json")
    }

    router.get("/simslot/{slot}/state.json") { _, context in
        try slotFileResponse(root: root, context: context, fileName: "state.json")
    }

    // No-ops in v1: nothing to "open" remotely (you start swift-mujoco processes yourself),
    // and there's no control channel back into a running process yet for stop/pause/step/reset.
    router.post("/ctl/sim-open") { _, _ -> EditedResponse<SimControlResponse> in
        EditedResponse(status: HTTPResponse.Status(code: 501),
                      response: SimControlResponse(ok: false, control: nil))
    }

    router.post("/ctl/sim-stop") { _, _ -> SimControlResponse in
        SimControlResponse(ok: true, control: nil)
    }

    router.post("/ctl/sim-cmd") { _, _ -> SimControlResponse in
        SimControlResponse(ok: true, control: SimControl(paused: nil, step: nil, reset: nil))
    }

    return router
}

private func slotFileResponse<Context: RequestContext>(root: URL, context: Context,
                                                        fileName: String) throws -> Response {
    guard let slot = context.parameters.get("slot") else { throw HTTPError(.badRequest) }
    let fileURL = root.appendingPathComponent(slot).appendingPathComponent(fileName)
    guard let data = try? Data(contentsOf: fileURL) else { throw HTTPError(.notFound) }
    return Response(status: .ok, headers: [.contentType: "application/json"],
                    body: .init(byteBuffer: ByteBuffer(data: data)))
}
```

- [ ] **Step 5: Wire the real entry point**

Replace the contents of `Sources/wendy-worldsim-server/WendyWorldSimServer.swift`:

```swift
import Foundation
import Hummingbird
import WendyMuJoCo

@main
struct WendyWorldSimServer {
    static func main() async throws {
        let env = ProcessInfo.processInfo.environment
        let port = env["PORT"].flatMap(Int.init) ?? 8788
        let router = makeRouter(root: WorldSim.directory())
        let app = Application(router: router,
                              configuration: .init(address: .hostname("127.0.0.1", port: port)))
        try await app.runService()
    }
}
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `swift test --filter RoutesTests`
Expected: PASS (7 tests). If `Data(buffer:)` fails to resolve, confirm `import NIOFoundationCompat` is present in the test file — it supplies the `ByteBuffer`↔`Data` bridge.

- [ ] **Step 7: Run the full test suite**

Run: `PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig swift test --skip MujocoRLDemoTests`
Expected: PASS — no regressions in `MuJoCoTests`, `WendyMuJoCoTests`, `MuJoCoRLEnvTests`, `WendyWorldSimServerTests`.

- [ ] **Step 8: Commit**

```bash
git add Sources/wendy-worldsim-server Tests/WendyWorldSimServerTests
git commit -m "feat: implement wendy-worldsim-server's HTTP routes"
```

---

## Self-Review Notes

- **Spec coverage:** slot convention (Task 1), `WorldSimRecorder` (Task 2), `mujoco-demo` wiring (Task 3), server scaffold + schema types (Task 4), slot discovery + title cache (Task 5), routes incl. the documented no-op POSTs and 404/heartbeat edge cases (Task 6) — all covered. WebSocket push and real pause/step/reset control are explicitly out of scope per the design's Future Work section; no task implements them.
- **Placeholder scan:** none — every step has literal code, exact file paths, and exact run commands.
- **Type consistency:** `RunningSim(slot:file:title:)`, `SimRunningResponse(running:focus:)`, `SimListResponse(sims:current:)`, `SimControl(paused:step:reset:)`, `SimControlResponse(ok:control:)` are used identically across Tasks 4, 5, and 6. `WorldSimRecorder(dir:)` / `.record(model:data:title:frame:hud:level:)` matches between Task 2's implementation and Task 3's call site.
- **Dependency fix caught on review:** Task 6's `Routes.swift` (`ByteBuffer`) and `RoutesTests.swift` (`Data(buffer:)`) reference NIO symbols that Hummingbird only provides *transitively* — SPM requires the package that owns them to be declared explicitly. Task 4 now adds `swift-nio` as a top-level dependency and `NIOCore`/`NIOFoundationCompat` as explicit product dependencies on `WendyWorldSimServer`/`WendyWorldSimServerTests`, rather than assuming Hummingbird re-exports them.

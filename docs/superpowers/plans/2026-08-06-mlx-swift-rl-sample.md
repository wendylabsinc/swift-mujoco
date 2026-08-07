# MLX-Swift RL Sample Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `mujoco-rl-demo` executable to swift-mujoco that trains a continuous-control policy to balance a cartpole using two algorithms — REINFORCE and PPO — built on MLX-Swift, with parallel rollout collection across independent MuJoCo environments.

**Architecture:** A new MLX-free library target (`MuJoCoRLEnv`) holds the cartpole environment, the plain-`[Float]` policy-weight snapshot type, the hand-rolled (non-MLX) forward pass used during rollout, and parallel trajectory collection via `TaskGroup`. A new executable target (`MujocoRLDemo`, gated to macOS since MLX is Metal-only) holds the MLX `Module` definitions (policy net, value net) and the two trainers, which snapshot the MLX model's weights into the plain-value type before handing them to parallel rollout workers — so no MLX object ever crosses a thread/task boundary.

**Tech Stack:** Swift 6.1, existing `MuJoCo` target (this repo), `ml-explore/mlx-swift` 0.31.6+ (`MLX`, `MLXNN`, `MLXOptimizers` products), Swift Concurrency (`TaskGroup`) for parallel rollout collection.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-06-mlx-swift-rl-sample-design.md`.
- `MjModel`/`MjData` are not `Sendable` (see doc comments in `Sources/MuJoCo/MjModel.swift` / `MjData.swift`). Never share one instance across a `TaskGroup` boundary — each parallel rollout worker creates its own `CartpoleEnv` (and therefore its own `MjModel`/`MjData`) entirely inside its own child task.
- MLX (`MLX`/`MLXNN`/`MLXOptimizers`) is used only inside the `MujocoRLDemo` executable target, never inside `MuJoCoRLEnv`. Rollout-time action sampling in `MuJoCoRLEnv` is a hand-rolled Swift forward pass over a plain `PolicyWeights` snapshot — no MLX types appear in that target.
- The `mlx-swift` package dependency, the `MujocoRLDemo` target/product, and `MujocoRLDemoTests` are all declared inside `#if os(macOS)` in `Package.swift`, so Linux CI's manifest evaluation never mentions `mlx-swift` and never attempts to resolve or build it. `MuJoCoRLEnv` and `MuJoCoRLEnvTests` are declared unconditionally (cross-platform, no MLX dependency).
- `mlx-swift` requires macOS 14+, so `Package.swift`'s `platforms:` entry must be bumped from `.macOS(.v13)` to `.macOS(.v14)`. This is an unconditional change to the array literal (not itself wrapped in `#if os(macOS)`) — a `.macOS(...)` platform entry has no effect on Linux builds either way, so this is safe for the Linux CI job.
- Target-naming convention already established in this repo: the Swift module/target name is PascalCase (`MujocoDemo`), the product name (what `swift run` invokes) is kebab-case (`mujoco-demo`), and the source directory matches the product name. Follow the same pattern: target `MujocoRLDemo`, product `mujoco-rl-demo`, path `Sources/mujoco-rl-demo`.
- Build/test locally with: `export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig` (MuJoCo must already be installed per the repo's README; it is on this machine).
- **`mlx-swift` cannot execute at runtime under plain `swift build`/`swift test`/`swift run`.** This was discovered during Task 5 and independently confirmed by the controller: `mlx-swift`'s own README states "SwiftPM (command line) cannot build the Metal shaders so the ultimate build has to be done via Xcode." Plain `swift build`/`swift test`/`swift run` compile and link fine, but the moment any code actually executes an MLX operation, the process crashes with `MLX error: Failed to load the default metallib`. Only `xcodebuild` (which does compile Metal shaders) can produce a runnable/testable binary. Consequences, decided with the human partner:
  - Any task whose tests exercise real MLX operations (Tasks 6, 7, and the `GaussianPolicyTests` already merged in Task 5) must be verified with `xcodebuild test -scheme swift-mujoco-Package -destination 'platform=macOS'` (the whole scheme — `-only-testing:` filters were tried and did not reliably match Swift Testing's `@Test` identifiers, so run the full suite rather than one that silently executes 0 tests) instead of `swift test --filter ...`. `swift build`/`swift test --skip MujocoRLDemoTests` (see below) still verify everything else and remain part of each task's workflow — `xcodebuild test` is an *addition* for the MLX-touching tests specifically, not a replacement for the rest of the task's verification.
  - Task 8's end-to-end run uses `xcodebuild build -scheme mujoco-rl-demo -destination 'platform=macOS' -derivedDataPath <dir>` and then executes the resulting binary directly (`<dir>/Build/Products/Debug/mujoco-rl-demo reinforce`), not `swift run` — confirmed working: `xcodebuild build` bundles a real `default.metallib` next to the binary (inside `mlx-swift_Cmlx.bundle`), which plain `swift build` never produces.
  - This repo's macOS CI job's `swift test -v` step must gain `--skip MujocoRLDemoTests` (confirmed: this avoids the crash entirely, and the other 83 existing tests still run and pass under the flag). This is the one CI YAML change this plan now requires — it was originally scoped as "no CI changes needed," which this finding supersedes. `MujocoRLDemoTests` therefore never runs in CI; it's verified locally via `xcodebuild test` as part of each task's dispatch, and the README documents the same requirement for contributors.

---

### Task 1: `Package.swift` — new targets, MLX dependency, platform bump

**Files:**
- Modify: `Package.swift` (full-file replacement)

**Interfaces:**
- Produces: target `MuJoCoRLEnv` (library, deps: `MuJoCo`), target `MuJoCoRLEnvTests` (test, deps: `MuJoCoRLEnv`, `MuJoCo`) — both unconditional/cross-platform.
- Produces (macOS-only): target `MujocoRLDemo` (executable, deps: `MuJoCoRLEnv`, `MLX`, `MLXNN`, `MLXOptimizers`), product `mujoco-rl-demo`, target `MujocoRLDemoTests` (test, deps: `MujocoRLDemo`, `MuJoCoRLEnv`).
- Consumes: nothing from later tasks (this is the foundation).

- [ ] **Step 1: Replace `Package.swift` with the restructured manifest**

```swift
// swift-tools-version: 6.1
import PackageDescription

var targets: [Target] = [
    .systemLibrary(name: "CMuJoCo", path: "Sources/CMuJoCo", pkgConfig: "mujoco"),
    .target(
        name: "CMuJoCoGL",
        cSettings: [
            .define("GL_SILENCE_DEPRECATION", to: "1", .when(platforms: [.macOS]))
        ],
        linkerSettings: [
            .linkedFramework("OpenGL", .when(platforms: [.macOS]))
        ]
    ),
    .target(name: "MuJoCo", dependencies: ["CMuJoCo", "CMuJoCoGL"]),
    .executableTarget(name: "MujocoDemo", dependencies: ["MuJoCo"], path: "Sources/mujoco-demo"),
    .testTarget(name: "MuJoCoTests", dependencies: ["MuJoCo"]),
    .target(name: "WendyMuJoCo", dependencies: ["MuJoCo", "CMuJoCo"]),
    .testTarget(name: "WendyMuJoCoTests", dependencies: ["WendyMuJoCo", "MuJoCo"]),
    .target(name: "MuJoCoRLEnv", dependencies: ["MuJoCo"]),
    .testTarget(name: "MuJoCoRLEnvTests", dependencies: ["MuJoCoRLEnv", "MuJoCo"]),
]

var products: [Product] = [
    .library(name: "MuJoCo", targets: ["MuJoCo"]),
    .executable(name: "mujoco-demo", targets: ["MujocoDemo"]),
    .library(name: "WendyMuJoCo", targets: ["WendyMuJoCo"]),
]

var dependencies: [Package.Dependency] = []

#if os(macOS)
dependencies.append(.package(url: "https://github.com/ml-explore/mlx-swift", from: "0.31.6"))

targets.append(
    .executableTarget(
        name: "MujocoRLDemo",
        dependencies: [
            "MuJoCoRLEnv",
            .product(name: "MLX", package: "mlx-swift"),
            .product(name: "MLXNN", package: "mlx-swift"),
            .product(name: "MLXOptimizers", package: "mlx-swift"),
        ],
        path: "Sources/mujoco-rl-demo"
    )
)
targets.append(.testTarget(name: "MujocoRLDemoTests", dependencies: ["MujocoRLDemo", "MuJoCoRLEnv"]))
products.append(.executable(name: "mujoco-rl-demo", targets: ["MujocoRLDemo"]))
#endif

let package = Package(
    name: "swift-mujoco",
    platforms: [.macOS(.v14)],
    products: products,
    dependencies: dependencies,
    targets: targets
)
```

(`dependencies:` must precede `targets:` here — Swift requires labeled arguments in the initializer's declared order, and `Package.init` declares `dependencies` before `targets`.)

- [ ] **Step 2: Resolve and build**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
cd /Users/joannisorlandos/git/wendy/swift-mujoco
swift package resolve
swift build
```
Expected: `swift package resolve` fetches `mlx-swift` (this takes a while the first time — it vendors a prebuilt Metal/C++ core); `swift build` succeeds with no source files yet in `Sources/mujoco-rl-demo` other than what SwiftPM requires. Since the target has no `.swift` files yet, create a placeholder immediately so the build has something to compile:

```bash
mkdir -p Sources/mujoco-rl-demo
cat > Sources/mujoco-rl-demo/main.swift <<'EOF'
print("mujoco-rl-demo: scaffold")
EOF
mkdir -p Tests/MujocoRLDemoTests
cat > Tests/MujocoRLDemoTests/PlaceholderTests.swift <<'EOF'
import Testing

@Test func placeholder() {
    #expect(true)
}
EOF
mkdir -p Tests/MuJoCoRLEnvTests
cat > Tests/MuJoCoRLEnvTests/PlaceholderTests.swift <<'EOF'
import Testing

@Test func placeholder() {
    #expect(true)
}
EOF
mkdir -p Sources/MuJoCoRLEnv
cat > Sources/MuJoCoRLEnv/Placeholder.swift <<'EOF'
// Replaced by Task 2 — SwiftPM requires a library target to have at least
// one source file.
EOF
swift build
swift test
```
Expected: both commands succeed; the two placeholder tests pass. (Later tasks replace these placeholder files' content — the files themselves stay, since SwiftPM's default test target discovery requires at least one file to exist for `swift build`/`swift test` to succeed on a target with no sources yet, and the `MuJoCoRLEnv` library target needs a source file too, not just its test target.)

- [ ] **Step 3: Commit**

```bash
git add Package.swift Package.resolved Sources/mujoco-rl-demo Tests/MuJoCoRLEnvTests Tests/MujocoRLDemoTests
git commit -m "build: scaffold MuJoCoRLEnv and MujocoRLDemo targets for the MLX-Swift RL sample"
```

---

### Task 2: `CartpoleEnv` — the cartpole balance task

**Files:**
- Create: `Sources/MuJoCoRLEnv/CartpoleEnv.swift`
- Delete: `Sources/MuJoCoRLEnv/Placeholder.swift` (Task 1's scaffolding stand-in — no longer needed once this target has real source)
- Test: `Tests/MuJoCoRLEnvTests/CartpoleEnvTests.swift` (replaces the Task 1 placeholder)

**Interfaces:**
- Produces: `struct CartpoleObservation` (internal; fields `cartPosition, cartVelocity, poleAngle, poleAngularVelocity: Double`, computed `var asArray: [Float]`), `final class CartpoleEnv` (internal; `init()`, `func reset() -> CartpoleObservation`, `func step(action: Float) -> (observation: CartpoleObservation, reward: Float, done: Bool)`, `static let maxSteps = 500`, `static let cartPositionLimit: Double = 2.4`, `static let poleAngleLimit: Double = 0.2094395102393195`). Both are internal (not `public`) — `CartpoleEnv` is an implementation detail of rollout collection (Task 4), not part of `MuJoCoRLEnv`'s public API, so the test file uses `@testable import`.
- Consumes: `MuJoCo` (`MjModel`, `MjData`, `mjStep`, `mjResetData`) — already present in this repo.

- [ ] **Step 1: Write the failing tests**

```swift
// Tests/MuJoCoRLEnvTests/CartpoleEnvTests.swift
import Testing

@testable import MuJoCoRLEnv

@Test func resetStartsUprightAndCentered() {
    let env = CartpoleEnv()
    let obs = env.reset()
    #expect(obs.cartPosition == 0)
    #expect(obs.poleAngle == 0)
    #expect(obs.cartVelocity == 0)
    #expect(obs.poleAngularVelocity == 0)
}

@Test func zeroActionKeepsPoleUprightBriefly() {
    let env = CartpoleEnv()
    _ = env.reset()
    let (obs, reward, done) = env.step(action: 0)
    #expect(reward == 1.0)
    #expect(done == false)
    #expect(abs(obs.poleAngle) < 0.05)
}

@Test func sustainedLargeActionEventuallyTerminates() {
    let env = CartpoleEnv()
    _ = env.reset()
    var done = false
    var steps = 0
    while !done && steps < CartpoleEnv.maxSteps {
        let (_, _, isDone) = env.step(action: 1.0)
        done = isDone
        steps += 1
    }
    #expect(done == true)
    #expect(steps < CartpoleEnv.maxSteps)
}

@Test func episodeTerminatesAtMaxStepsUnderZeroAction() {
    let env = CartpoleEnv()
    _ = env.reset()
    var done = false
    var steps = 0
    while !done {
        let (_, _, isDone) = env.step(action: 0)
        done = isDone
        steps += 1
    }
    #expect(steps == CartpoleEnv.maxSteps)
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter CartpoleEnvTests
```
Expected: FAIL to compile — `CartpoleEnv` does not exist yet.

- [ ] **Step 3: Write `CartpoleEnv.swift`**

```swift
// Sources/MuJoCoRLEnv/CartpoleEnv.swift
import MuJoCo

struct CartpoleObservation {
    let cartPosition: Double
    let cartVelocity: Double
    let poleAngle: Double
    let poleAngularVelocity: Double

    var asArray: [Float] {
        [Float(cartPosition), Float(cartVelocity), Float(poleAngle), Float(poleAngularVelocity)]
    }
}

/// Cart-on-a-rail with an unactuated pole, balanced by a continuous force on
/// the cart. Not `Sendable` (it owns `MjModel`/`MjData`, neither of which
/// is) — each parallel rollout worker in `collectEpisode` constructs its own
/// instance rather than sharing one across tasks.
final class CartpoleEnv {
    static let maxSteps = 500
    static let cartPositionLimit: Double = 2.4
    // 12 degrees, matching the classic cartpole task's failure threshold.
    static let poleAngleLimit: Double = 0.2094395102393195

    private let model: MjModel
    private let data: MjData
    private var stepCount = 0

    init() {
        self.model = try! MjModel.load(xml: Self.xml)
        self.data = MjData(model)
    }

    func reset() -> CartpoleObservation {
        mjResetData(model, data)
        stepCount = 0
        return observation()
    }

    func step(action: Float) -> (observation: CartpoleObservation, reward: Float, done: Bool) {
        data.setCtrl([Double(action)])
        mjStep(model, data)
        stepCount += 1
        let obs = observation()
        let outOfBounds =
            abs(obs.cartPosition) > Self.cartPositionLimit || abs(obs.poleAngle) > Self.poleAngleLimit
        let done = outOfBounds || stepCount >= Self.maxSteps
        return (obs, 1.0, done)
    }

    private func observation() -> CartpoleObservation {
        CartpoleObservation(
            cartPosition: data.qpos(at: 0),
            cartVelocity: data.qvel(at: 0),
            poleAngle: data.qpos(at: 1),
            poleAngularVelocity: data.qvel(at: 1)
        )
    }

    // qpos[0]/qvel[0] is the slider (cart); qpos[1]/qvel[1] is the hinge
    // (pole) — MuJoCo assigns DOF addresses in kinematic-tree declaration
    // order, and the slider joint is declared before descending into the
    // pole body. State always resets to upright/centered; exploration comes
    // from the policy's Gaussian action noise, not initial-state randomization.
    private static let xml = """
        <mujoco>
          <option timestep="0.02" gravity="0 0 -9.81"/>
          <worldbody>
            <body name="cart" pos="0 0 0">
              <joint name="slider" type="slide" axis="1 0 0" range="-3 3" damping="0.1"/>
              <geom name="cart_geom" type="box" size="0.1 0.1 0.05" rgba="0.2 0.6 0.9 1" mass="1.0"/>
              <body name="pole" pos="0 0 0.05">
                <joint name="hinge" type="hinge" axis="0 1 0" damping="0.01"/>
                <geom name="pole_geom" type="capsule" fromto="0 0 0 0 0 0.5" size="0.02" mass="0.1" rgba="0.9 0.3 0.2 1"/>
              </body>
            </body>
          </worldbody>
          <actuator>
            <motor name="slide_motor" joint="slider" gear="20" ctrlrange="-1 1"/>
          </actuator>
        </mujoco>
        """
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter CartpoleEnvTests
```
Expected: PASS (all 4 tests).

- [ ] **Step 5: Commit**

```bash
git rm Sources/MuJoCoRLEnv/Placeholder.swift
git add Sources/MuJoCoRLEnv/CartpoleEnv.swift Tests/MuJoCoRLEnvTests/CartpoleEnvTests.swift
git commit -m "feat: add CartpoleEnv balance task to MuJoCoRLEnv"
```

---

### Task 3: Policy weight snapshot + hand-rolled forward pass + Gaussian sampling

**Files:**
- Create: `Sources/MuJoCoRLEnv/PolicyWeights.swift`
- Test: `Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift`

**Interfaces:**
- Produces (all `public`, no MLX dependency): `struct PolicyWeights: Sendable` (`w1, b1, w2, b2: [Float]`, `logStd: Float`, `inputDimensions, hiddenDimensions: Int`, memberwise `public init`), `func policyForward(_ weights: PolicyWeights, observation: [Float]) -> (mean: Float, std: Float)`, `func gaussianLogProb(action: Float, mean: Float, std: Float) -> Float`, `struct SplitMix64: RandomNumberGenerator` (`init(seed: UInt64)`, `mutating func next() -> UInt64`), `func sampleGaussian(mean: Float, std: Float, using rng: inout SplitMix64) -> Float`.
- Consumes: nothing beyond the standard library.

- [ ] **Step 1: Write the failing tests**

```swift
// Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift
import Testing

@testable import MuJoCoRLEnv

@Test func forwardPassMatchesHandComputedValue() {
    // hiddenDimensions=2, inputDimensions=2, w1 = identity, b1 = 0,
    // w2 = [1, 1], b2 = 0, logStd = 0.
    // hidden = tanh([1, 1]) ≈ [0.7615941559557649, 0.7615941559557649]
    // mean = hidden[0] + hidden[1] ≈ 1.5231883119115298 ; std = exp(0) = 1
    let weights = PolicyWeights(
        w1: [1, 0, 0, 1], b1: [0, 0], w2: [1, 1], b2: [0], logStd: 0,
        inputDimensions: 2, hiddenDimensions: 2
    )
    let (mean, std) = policyForward(weights, observation: [1, 1])
    #expect(abs(mean - 1.5231883) < 1e-4)
    #expect(abs(std - 1.0) < 1e-6)
}

@Test func gaussianLogProbMatchesClosedForm() {
    // logProb(0 | mean=0, std=1) = -0.5 * log(2*pi) ≈ -0.9189385332
    let logProb = gaussianLogProb(action: 0, mean: 0, std: 1)
    #expect(abs(logProb - (-0.9189385)) < 1e-4)
}

@Test func splitMix64IsDeterministicGivenASeed() {
    var rngA = SplitMix64(seed: 42)
    var rngB = SplitMix64(seed: 42)
    #expect(rngA.next() == rngB.next())
    #expect(rngA.next() == rngB.next())
}

@Test func sampleGaussianIsReproducibleForTheSameSeed() {
    var rngA = SplitMix64(seed: 7)
    var rngB = SplitMix64(seed: 7)
    let a = sampleGaussian(mean: 0, std: 1, using: &rngA)
    let b = sampleGaussian(mean: 0, std: 1, using: &rngB)
    #expect(a == b)
}

@Test func sampleGaussianWithZeroStdReturnsTheMean() {
    var rng = SplitMix64(seed: 1)
    let action = sampleGaussian(mean: 3.5, std: 0, using: &rng)
    #expect(action == 3.5)
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter PolicyWeightsTests
```
Expected: FAIL to compile — none of these types exist yet.

- [ ] **Step 3: Write `PolicyWeights.swift`**

```swift
// Sources/MuJoCoRLEnv/PolicyWeights.swift
#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif

/// A plain-value snapshot of a Gaussian policy's weights: `Linear(in→hidden)
/// → tanh → Linear(hidden→1)` for the action mean, plus a scalar log-std.
/// `Sendable` by construction (all stored properties are value types), so it
/// can cross a `TaskGroup` boundary into parallel rollout workers with no
/// MLX object ever doing the same.
public struct PolicyWeights: Sendable {
    /// Row-major [hiddenDimensions x inputDimensions].
    public let w1: [Float]
    public let b1: [Float]
    /// Row-major [1 x hiddenDimensions].
    public let w2: [Float]
    public let b2: [Float]
    public let logStd: Float
    public let inputDimensions: Int
    public let hiddenDimensions: Int

    public init(
        w1: [Float], b1: [Float], w2: [Float], b2: [Float], logStd: Float,
        inputDimensions: Int, hiddenDimensions: Int
    ) {
        self.w1 = w1
        self.b1 = b1
        self.w2 = w2
        self.b2 = b2
        self.logStd = logStd
        self.inputDimensions = inputDimensions
        self.hiddenDimensions = hiddenDimensions
    }
}

/// The rollout-time policy forward pass. Deliberately not MLX: this runs
/// inside parallel `TaskGroup` workers, and MLX's thread-safety for
/// concurrent forward passes across OS threads is unverified. Gradients are
/// only ever needed at training time (see `MujocoRLDemo`), so this plain
/// arithmetic version is sufficient here.
public func policyForward(_ weights: PolicyWeights, observation: [Float]) -> (mean: Float, std: Float) {
    precondition(observation.count == weights.inputDimensions)
    var hidden = [Float](repeating: 0, count: weights.hiddenDimensions)
    for h in 0..<weights.hiddenDimensions {
        var sum = weights.b1[h]
        for i in 0..<weights.inputDimensions {
            sum += weights.w1[h * weights.inputDimensions + i] * observation[i]
        }
        hidden[h] = tanh(sum)
    }
    var mean = weights.b2[0]
    for h in 0..<weights.hiddenDimensions {
        mean += weights.w2[h] * hidden[h]
    }
    return (mean, exp(weights.logStd))
}

public func gaussianLogProb(action: Float, mean: Float, std: Float) -> Float {
    let variance = std * std
    return -0.5 * log(2 * Float.pi * variance) - (action - mean) * (action - mean) / (2 * variance)
}

/// A small seedable RNG so rollout workers get independent, reproducible
/// action-sampling streams instead of contending on a shared global RNG.
public struct SplitMix64: RandomNumberGenerator {
    private var state: UInt64

    public init(seed: UInt64) {
        self.state = seed
    }

    public mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }
}

public func sampleGaussian(mean: Float, std: Float, using rng: inout SplitMix64) -> Float {
    guard std > 0 else { return mean }
    // Box-Muller.
    let u1 = Float.random(in: Float.ulpOfOne..<1, using: &rng)
    let u2 = Float.random(in: 0..<1, using: &rng)
    let z = sqrt(-2 * log(u1)) * cos(2 * Float.pi * u2)
    return mean + std * z
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter PolicyWeightsTests
```
Expected: PASS (all 5 tests).

- [ ] **Step 5: Commit**

```bash
git add Sources/MuJoCoRLEnv/PolicyWeights.swift Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift
git commit -m "feat: add hand-rolled Gaussian policy forward pass for rollout workers"
```

---

### Task 4: `Trajectory`, discounted returns, and parallel rollout collection

**Files:**
- Create: `Sources/MuJoCoRLEnv/Rollout.swift`
- Test: `Tests/MuJoCoRLEnvTests/RolloutTests.swift`

**Interfaces:**
- Produces (all `public`): `struct Trajectory: Sendable` (`var observations: [[Float]]`, `var actions: [Float]`, `var logProbs: [Float]`, `var rewards: [Float]`, `public init(observations: [[Float]] = [], actions: [Float] = [], logProbs: [Float] = [], rewards: [Float] = [])`), `func discountedReturns(rewards: [Float], gamma: Float) -> [Float]`, `func collectEpisode(weights: PolicyWeights, seed: UInt64) -> Trajectory`, `func collectBatch(weights: PolicyWeights, episodeCount: Int, baseSeed: UInt64) async -> [Trajectory]`.
- Consumes: `PolicyWeights`, `policyForward`, `sampleGaussian`, `gaussianLogProb`, `SplitMix64` (Task 3); `CartpoleEnv` (Task 2, internal to this same target).

- [ ] **Step 1: Write the failing tests**

```swift
// Tests/MuJoCoRLEnvTests/RolloutTests.swift
import Testing

@testable import MuJoCoRLEnv

@Test func discountedReturnsMatchHandComputedValues() {
    // rewards = [1, 1, 1], gamma = 0.5
    // r[2] = 1 ; r[1] = 1 + 0.5*1 = 1.5 ; r[0] = 1 + 0.5*1.5 = 1.75
    let returns = discountedReturns(rewards: [1, 1, 1], gamma: 0.5)
    #expect(returns.count == 3)
    #expect(abs(returns[0] - 1.75) < 1e-6)
    #expect(abs(returns[1] - 1.5) < 1e-6)
    #expect(abs(returns[2] - 1.0) < 1e-6)
}

private let zeroWeightPolicy = PolicyWeights(
    w1: [Float](repeating: 0, count: 4 * 32), b1: [Float](repeating: 0, count: 32),
    w2: [Float](repeating: 0, count: 32), b2: [0], logStd: 0,
    inputDimensions: 4, hiddenDimensions: 32
)

@Test func collectEpisodeProducesConsistentArrayLengths() {
    let trajectory = collectEpisode(weights: zeroWeightPolicy, seed: 1)
    #expect(trajectory.observations.count > 0)
    #expect(trajectory.observations.count <= CartpoleEnv.maxSteps)
    #expect(trajectory.actions.count == trajectory.observations.count)
    #expect(trajectory.logProbs.count == trajectory.observations.count)
    #expect(trajectory.rewards.count == trajectory.observations.count)
    #expect(trajectory.observations.allSatisfy { $0.count == 4 })
}

@Test func collectBatchRunsEpisodesInParallel() async {
    let trajectories = await collectBatch(weights: zeroWeightPolicy, episodeCount: 4, baseSeed: 100)
    #expect(trajectories.count == 4)
    for trajectory in trajectories {
        #expect(trajectory.observations.count > 0)
    }
}

@Test func collectEpisodeIsDeterministicForAFixedSeed() {
    let a = collectEpisode(weights: zeroWeightPolicy, seed: 55)
    let b = collectEpisode(weights: zeroWeightPolicy, seed: 55)
    #expect(a.actions == b.actions)
    #expect(a.rewards == b.rewards)
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter RolloutTests
```
Expected: FAIL to compile — `Trajectory`/`discountedReturns`/`collectEpisode`/`collectBatch` don't exist yet.

- [ ] **Step 3: Write `Rollout.swift`**

```swift
// Sources/MuJoCoRLEnv/Rollout.swift

public struct Trajectory: Sendable {
    public var observations: [[Float]]
    public var actions: [Float]
    public var logProbs: [Float]
    public var rewards: [Float]

    public init(
        observations: [[Float]] = [], actions: [Float] = [], logProbs: [Float] = [],
        rewards: [Float] = []
    ) {
        self.observations = observations
        self.actions = actions
        self.logProbs = logProbs
        self.rewards = rewards
    }
}

/// Reward-to-go with discount `gamma`, computed backward through the episode.
public func discountedReturns(rewards: [Float], gamma: Float) -> [Float] {
    var returns = [Float](repeating: 0, count: rewards.count)
    var runningReturn: Float = 0
    for t in stride(from: rewards.count - 1, through: 0, by: -1) {
        runningReturn = rewards[t] + gamma * runningReturn
        returns[t] = runningReturn
    }
    return returns
}

/// Runs one full episode against a fresh `CartpoleEnv`, sampling actions
/// from `weights` via the hand-rolled forward pass. Safe to call
/// concurrently from multiple tasks: nothing here is shared mutable state,
/// and the `CartpoleEnv` this function creates never leaves it.
public func collectEpisode(weights: PolicyWeights, seed: UInt64) -> Trajectory {
    var rng = SplitMix64(seed: seed)
    let env = CartpoleEnv()
    var trajectory = Trajectory()
    var observation = env.reset()
    while true {
        let observationArray = observation.asArray
        let (mean, std) = policyForward(weights, observation: observationArray)
        let action = sampleGaussian(mean: mean, std: std, using: &rng)
        let logProb = gaussianLogProb(action: action, mean: mean, std: std)
        let (nextObservation, reward, done) = env.step(action: action)

        trajectory.observations.append(observationArray)
        trajectory.actions.append(action)
        trajectory.logProbs.append(logProb)
        trajectory.rewards.append(reward)

        observation = nextObservation
        if done { break }
    }
    return trajectory
}

/// Fans out `episodeCount` independent episodes across a `TaskGroup`. Each
/// worker gets a distinct seed (`baseSeed + index`) so parallel workers don't
/// duplicate the same action sequence.
public func collectBatch(weights: PolicyWeights, episodeCount: Int, baseSeed: UInt64) async -> [Trajectory] {
    await withTaskGroup(of: Trajectory.self) { group in
        for index in 0..<episodeCount {
            group.addTask {
                collectEpisode(weights: weights, seed: baseSeed &+ UInt64(index))
            }
        }
        var trajectories: [Trajectory] = []
        trajectories.reserveCapacity(episodeCount)
        for await trajectory in group {
            trajectories.append(trajectory)
        }
        return trajectories
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter RolloutTests
```
Expected: PASS (all 4 tests).

- [ ] **Step 5: Commit**

```bash
git add Sources/MuJoCoRLEnv/Rollout.swift Tests/MuJoCoRLEnvTests/RolloutTests.swift
git commit -m "feat: add discounted returns and parallel rollout collection"
```

---

### Task 5: `GaussianPolicy` MLX module + weight-snapshot bridge + shared log-prob helper

**Files:**
- Create: `Sources/mujoco-rl-demo/GaussianPolicy.swift`
- Create: `Sources/mujoco-rl-demo/GaussianLogProb.swift`
- Test: `Tests/MujocoRLDemoTests/GaussianPolicyTests.swift` (replaces the Task 1 placeholder)

**Interfaces:**
- Produces (internal to `MujocoRLDemo`, no `public` needed — this is an executable target): `final class GaussianPolicy: Module` (`@ModuleInfo var fc1: Linear`, `@ModuleInfo var fc2: Linear`, `var logStd: MLXArray`, `init(observationDimensions: Int, hiddenDimensions: Int)`, `func callAsFunction(_ x: MLXArray) -> MLXArray`), `extension GaussianPolicy { func snapshot() -> PolicyWeights }`, `func gaussianLogProbMLX(actions: MLXArray, mean: MLXArray, std: MLXArray) -> MLXArray`. `gaussianLogProbMLX` is the MLX-array counterpart of `MuJoCoRLEnv`'s scalar `gaussianLogProb` — both `ReinforceTrainer` (Task 6) and `PPOTrainer` (Task 7) call this shared helper for their loss closures instead of each inlining the formula.
- Consumes: `PolicyWeights`, `policyForward`, `gaussianLogProb` (from `MuJoCoRLEnv`, Task 3); `Module`, `Linear`, `@ModuleInfo` (MLXNN); `MLXArray`, `tanh`, `exp`, `log`, `square` (MLX).

- [ ] **Step 1: Write the failing test**

```swift
// Tests/MujocoRLDemoTests/GaussianPolicyTests.swift
import MLX
import Testing

@testable import MujocoRLDemo
import MuJoCoRLEnv

@Test func snapshotMatchesLiveMLXForwardPass() {
    let policy = GaussianPolicy(observationDimensions: 4, hiddenDimensions: 8)
    let weights = policy.snapshot()

    let observation: [Float] = [0.1, -0.2, 0.3, -0.4]
    let mlxOutput = policy(MLXArray(observation, [1, 4]))
    let mlxMean = mlxOutput.item(Float.self)

    let (handRolledMean, _) = policyForward(weights, observation: observation)

    #expect(abs(mlxMean - handRolledMean) < 1e-4)
}

@Test func snapshotShapesMatchDeclaredDimensions() {
    let policy = GaussianPolicy(observationDimensions: 4, hiddenDimensions: 8)
    let weights = policy.snapshot()
    #expect(weights.w1.count == 8 * 4)
    #expect(weights.b1.count == 8)
    #expect(weights.w2.count == 8)
    #expect(weights.b2.count == 1)
    #expect(weights.inputDimensions == 4)
    #expect(weights.hiddenDimensions == 8)
}

@Test func gaussianLogProbMLXMatchesScalarImplementation() {
    let mean: Float = 0.5
    let std: Float = 1.2
    let action: Float = -0.3
    let mlxResult = gaussianLogProbMLX(
        actions: MLXArray([action], [1, 1]),
        mean: MLXArray([mean], [1, 1]),
        std: MLXArray([std], [1, 1])
    ).item(Float.self)
    let scalarResult = gaussianLogProb(action: action, mean: mean, std: std)
    #expect(abs(mlxResult - scalarResult) < 1e-4)
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter GaussianPolicyTests
```
Expected: FAIL to compile — `GaussianPolicy` doesn't exist yet.

- [ ] **Step 3: Write `GaussianLogProb.swift`**

```swift
// Sources/mujoco-rl-demo/GaussianLogProb.swift
import MLX

/// MLX-array Gaussian log-probability — the array counterpart of
/// `MuJoCoRLEnv`'s scalar `gaussianLogProb`, used inside both trainers'
/// gradient-tracked loss closures so the formula lives in exactly one place.
func gaussianLogProbMLX(actions: MLXArray, mean: MLXArray, std: MLXArray) -> MLXArray {
    let variance = square(std)
    return -0.5 * log(2 * Float.pi * variance) - square(actions - mean) / (2 * variance)
}
```

- [ ] **Step 4: Write `GaussianPolicy.swift`**

```swift
// Sources/mujoco-rl-demo/GaussianPolicy.swift
import MLX
import MLXNN
import MuJoCoRLEnv

/// `Linear(observationDimensions → hiddenDimensions) → tanh →
/// Linear(hiddenDimensions → 1)` for the action mean, plus a learned scalar
/// log-std. Used only for training (gradient computation via `valueAndGrad`)
/// — rollout-time action sampling uses `snapshot()` and the MLX-free
/// `policyForward` in `MuJoCoRLEnv` instead.
final class GaussianPolicy: Module {
    @ModuleInfo var fc1: Linear
    @ModuleInfo var fc2: Linear
    var logStd: MLXArray

    let observationDimensions: Int
    let hiddenDimensions: Int

    init(observationDimensions: Int, hiddenDimensions: Int) {
        self.observationDimensions = observationDimensions
        self.hiddenDimensions = hiddenDimensions
        self.fc1 = Linear(observationDimensions, hiddenDimensions)
        self.fc2 = Linear(hiddenDimensions, 1)
        self.logStd = MLXArray(Float(0.0))
    }

    func callAsFunction(_ x: MLXArray) -> MLXArray {
        fc2(tanh(fc1(x)))
    }
}

extension GaussianPolicy {
    /// Extracts a plain-value weight snapshot for rollout workers. `Linear`
    /// stores `weight` as `[outputDimensions, inputDimensions]` row-major —
    /// the same layout `policyForward` expects — and is always constructed
    /// with `bias: true` (the default) here, so `.bias!` is safe.
    func snapshot() -> PolicyWeights {
        PolicyWeights(
            w1: fc1.weight.asArray(Float.self),
            b1: fc1.bias!.asArray(Float.self),
            w2: fc2.weight.asArray(Float.self),
            b2: fc2.bias!.asArray(Float.self),
            logStd: logStd.item(Float.self),
            inputDimensions: observationDimensions,
            hiddenDimensions: hiddenDimensions
        )
    }
}
```

- [ ] **Step 5: Run the tests to verify they pass**

This exercises real MLX operations — plain `swift test` compiles it but crashes at runtime (`mlx-swift` cannot run under SwiftPM's CLI build; see Global Constraints). Use `xcodebuild test` instead:

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
xcodebuild test -scheme swift-mujoco-Package -destination 'platform=macOS'
```
Expected: `** TEST SUCCEEDED **`, with all three of this task's tests passing (run via the whole scheme — `-only-testing:` filters don't reliably match Swift Testing's `@Test` identifiers). If `snapshotMatchesLiveMLXForwardPass` fails on the flattening-order assumption, the fix is in `policyForward` (Task 3) or here, not in the test.

- [ ] **Step 6: Commit**

```bash
git add Sources/mujoco-rl-demo/GaussianPolicy.swift Sources/mujoco-rl-demo/GaussianLogProb.swift Tests/MujocoRLDemoTests/GaussianPolicyTests.swift
git commit -m "feat: add GaussianPolicy MLX module, rollout-weight snapshotting, and shared log-prob helper"
```

---

### Task 6: `ReinforceTrainer` and the `reinforce` CLI path

**Files:**
- Create: `Sources/mujoco-rl-demo/ReinforceTrainer.swift`
- Modify: `Sources/mujoco-rl-demo/main.swift` (replace placeholder content)
- Test: `Tests/MujocoRLDemoTests/ReinforceTrainerTests.swift`

**Interfaces:**
- Produces: `final class ReinforceTrainer` (`let policy: GaussianPolicy`, `init(observationDimensions: Int, hiddenDimensions: Int, learningRate: Float, gamma: Float)`, `@discardableResult func trainStep(trajectories: [Trajectory]) -> Float`).
- Consumes: `GaussianPolicy`, `gaussianLogProbMLX` (Task 5); `Trajectory`, `discountedReturns`, `collectBatch` (Task 4); `Adam`, `valueAndGrad`, `MLXArray`, `exp` (MLX/MLXOptimizers).

- [ ] **Step 1: Write the failing test**

This test checks the optimization mechanics (loss goes down against a fixed synthetic batch) rather than whether cartpole gets solved — that's stochastic and belongs to Task 8's manual verification, not a fast deterministic unit test.

```swift
// Tests/MujocoRLDemoTests/ReinforceTrainerTests.swift
import Testing

@testable import MujocoRLDemo
import MuJoCoRLEnv

@Test func trainStepReducesLossOnAFixedBatch() {
    let trainer = ReinforceTrainer(
        observationDimensions: 4, hiddenDimensions: 8, learningRate: 1e-2, gamma: 0.99
    )
    let trajectories = [
        Trajectory(
            observations: (0..<20).map { i in [Float(i) * 0.01, 0, 0.02, 0] },
            actions: (0..<20).map { i in Float(i) * 0.05 - 0.5 },
            logProbs: [Float](repeating: 0, count: 20),
            rewards: [Float](repeating: 1, count: 20)
        )
    ]

    let firstLoss = trainer.trainStep(trajectories: trajectories)
    var lastLoss = firstLoss
    for _ in 0..<19 {
        lastLoss = trainer.trainStep(trajectories: trajectories)
    }

    #expect(lastLoss < firstLoss)
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter ReinforceTrainerTests
```
Expected: FAIL to compile — `ReinforceTrainer` doesn't exist yet.

- [ ] **Step 3: Write `ReinforceTrainer.swift`**

```swift
// Sources/mujoco-rl-demo/ReinforceTrainer.swift
import MLX
import MLXOptimizers
import MuJoCoRLEnv

final class ReinforceTrainer {
    let policy: GaussianPolicy
    private let optimizer: Adam
    private let gamma: Float
    private let observationDimensions: Int

    init(observationDimensions: Int, hiddenDimensions: Int, learningRate: Float, gamma: Float) {
        self.observationDimensions = observationDimensions
        self.policy = GaussianPolicy(observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions)
        self.optimizer = Adam(learningRate: learningRate)
        self.gamma = gamma
        eval(policy)
    }

    /// Discounted returns with a batch-mean baseline; one Adam step over the
    /// whole batch.
    @discardableResult
    func trainStep(trajectories: [Trajectory]) -> Float {
        var flatObservations: [Float] = []
        var flatActions: [Float] = []
        var flatReturns: [Float] = []
        for trajectory in trajectories {
            let returns = discountedReturns(rewards: trajectory.rewards, gamma: gamma)
            flatReturns.append(contentsOf: returns)
            for observation in trajectory.observations {
                flatObservations.append(contentsOf: observation)
            }
            flatActions.append(contentsOf: trajectory.actions)
        }
        let count = flatActions.count
        let baseline = flatReturns.reduce(0, +) / Float(flatReturns.count)
        let advantages = flatReturns.map { $0 - baseline }

        let observationsArray = MLXArray(flatObservations, [count, observationDimensions])
        let actionsArray = MLXArray(flatActions, [count, 1])
        let advantagesArray = MLXArray(advantages, [count, 1])

        func loss(model: GaussianPolicy, args: (MLXArray, MLXArray, MLXArray)) -> [MLXArray] {
            let (observations, actions, advantages) = args
            let mean = model(observations)
            let std = exp(model.logStd)
            let logProb = gaussianLogProbMLX(actions: actions, mean: mean, std: std)
            let lossValue = -(logProb * advantages).mean()
            return [lossValue]
        }

        let lossAndGrad = valueAndGrad(model: policy, loss)
        let (losses, gradients) = lossAndGrad(policy, (observationsArray, actionsArray, advantagesArray))
        optimizer.update(model: policy, gradients: gradients)
        eval(policy, optimizer)
        return losses[0].item(Float.self)
    }
}
```

- [ ] **Step 4: Run the test to verify it passes**

This test executes real MLX operations (`valueAndGrad`, `Adam.update`) — plain `swift test` compiles it but crashes at runtime (`mlx-swift` cannot run under SwiftPM's CLI build; see Global Constraints). Use `xcodebuild test` instead, which does compile the Metal shaders `mlx-swift` needs:

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
xcodebuild test -scheme swift-mujoco-Package -destination 'platform=macOS'
```
Expected: `** TEST SUCCEEDED **`, with `trainStepReducesLossOnAFixedBatch` (and every other test in the suite) passing. This runs the whole suite rather than a filtered subset — `-only-testing:` filters don't reliably match Swift Testing's `@Test` identifiers, and a filter that silently matches nothing still reports success.

- [ ] **Step 5: Wire up the `reinforce` CLI path in `main.swift`**

```swift
// Sources/mujoco-rl-demo/main.swift
import MuJoCoRLEnv

let algorithm = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "reinforce"
let observationDimensions = 4
let hiddenDimensions = 32
let gamma: Float = 0.99
let episodesPerBatch = 16
let iterations = 200

func meanReturn(_ trajectories: [Trajectory]) -> Float {
    let totals = trajectories.map { $0.rewards.reduce(0, +) }
    return totals.reduce(0, +) / Float(totals.count)
}

switch algorithm {
case "reinforce":
    let trainer = ReinforceTrainer(
        observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
        learningRate: 3e-3, gamma: gamma
    )
    print("Training REINFORCE on cartpole balance (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
    for iteration in 0..<iterations {
        let weights = trainer.policy.snapshot()
        let trajectories = await collectBatch(
            weights: weights, episodeCount: episodesPerBatch, baseSeed: UInt64(iteration) &* 1_000_003
        )
        let loss = trainer.trainStep(trajectories: trajectories)
        print("iter \(iteration): mean return \(meanReturn(trajectories))  (loss \(loss))")
    }
default:
    print("Unknown algorithm '\(algorithm)'. Usage: mujoco-rl-demo [reinforce|ppo]")
}
```

- [ ] **Step 6: Build**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift build
```
Expected: builds cleanly (the `ppo` case doesn't exist yet, but `default` covers it, so this compiles even before Task 7).

- [ ] **Step 7: Commit**

```bash
git add Sources/mujoco-rl-demo/ReinforceTrainer.swift Sources/mujoco-rl-demo/main.swift Tests/MujocoRLDemoTests/ReinforceTrainerTests.swift
git commit -m "feat: add ReinforceTrainer and wire up the reinforce CLI path"
```

---

### Task 7: `ValueNetwork`, `PPOTrainer`, and the `ppo` CLI path

**Files:**
- Create: `Sources/mujoco-rl-demo/PPOTrainer.swift`
- Modify: `Sources/mujoco-rl-demo/main.swift` (add the `ppo` case)
- Test: `Tests/MujocoRLDemoTests/PPOTrainerTests.swift`

**Interfaces:**
- Produces: `final class ValueNetwork: Module` (internal to this file), `final class PPOTrainer` (`let policy: GaussianPolicy`, `init(observationDimensions: Int, hiddenDimensions: Int, policyLearningRate: Float, valueLearningRate: Float, gamma: Float, clipEpsilon: Float, epochs: Int)`, `@discardableResult func trainStep(trajectories: [Trajectory]) -> (policyLoss: Float, valueLoss: Float)`).
- Consumes: `GaussianPolicy`, `gaussianLogProbMLX` (Task 5); `Trajectory`, `discountedReturns`, `collectBatch` (Task 4); `Adam`, `valueAndGrad`, `MLXArray`, `exp`, `square`, `clip`, `minimum` (MLX/MLXOptimizers); `Module`, `Linear`, `@ModuleInfo` (MLXNN).

- [ ] **Step 1: Write the failing test**

```swift
// Tests/MujocoRLDemoTests/PPOTrainerTests.swift
import Testing

@testable import MujocoRLDemo
import MuJoCoRLEnv

@Test func trainStepReducesValueLossOnAFixedBatch() {
    let trainer = PPOTrainer(
        observationDimensions: 4, hiddenDimensions: 8, policyLearningRate: 1e-2,
        valueLearningRate: 1e-1, gamma: 0.99, clipEpsilon: 0.2, epochs: 4
    )
    let observations = (0..<20).map { i in [Float(i) * 0.01, 0, 0.02, 0] }
    let actions = (0..<20).map { i in Float(i) * 0.05 - 0.5 }
    let logProbs = observations.indices.map { i in
        gaussianLogProb(action: actions[i], mean: 0, std: 1)
    }
    let trajectories = [
        Trajectory(
            observations: observations, actions: actions, logProbs: logProbs,
            rewards: [Float](repeating: 1, count: 20)
        )
    ]

    let (_, firstValueLoss) = trainer.trainStep(trajectories: trajectories)
    var lastValueLoss = firstValueLoss
    for _ in 0..<19 {
        (_, lastValueLoss) = trainer.trainStep(trajectories: trajectories)
    }

    #expect(lastValueLoss < firstValueLoss)
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter PPOTrainerTests
```
Expected: FAIL to compile — `PPOTrainer` doesn't exist yet.

- [ ] **Step 3: Write `PPOTrainer.swift`**

```swift
// Sources/mujoco-rl-demo/PPOTrainer.swift
import MLX
import MLXNN
import MLXOptimizers
import MuJoCoRLEnv

final class ValueNetwork: Module {
    @ModuleInfo var fc1: Linear
    @ModuleInfo var fc2: Linear

    init(observationDimensions: Int, hiddenDimensions: Int) {
        self.fc1 = Linear(observationDimensions, hiddenDimensions)
        self.fc2 = Linear(hiddenDimensions, 1)
    }

    func callAsFunction(_ x: MLXArray) -> MLXArray {
        fc2(tanh(fc1(x)))
    }
}

/// Minimal clipped-surrogate PPO: no GAE, a plain per-state value baseline,
/// full-batch (no minibatching) across a fixed number of epochs per
/// iteration. The "old" log-probs are the ones `collectBatch` already
/// recorded at rollout time using this same weight snapshot, so no separate
/// old-policy pass is needed.
final class PPOTrainer {
    let policy: GaussianPolicy
    private let valueNetwork: ValueNetwork
    private let policyOptimizer: Adam
    private let valueOptimizer: Adam
    private let gamma: Float
    private let clipEpsilon: Float
    private let epochs: Int
    private let observationDimensions: Int

    init(
        observationDimensions: Int, hiddenDimensions: Int, policyLearningRate: Float,
        valueLearningRate: Float, gamma: Float, clipEpsilon: Float, epochs: Int
    ) {
        self.observationDimensions = observationDimensions
        self.policy = GaussianPolicy(observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions)
        self.valueNetwork = ValueNetwork(observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions)
        self.policyOptimizer = Adam(learningRate: policyLearningRate)
        self.valueOptimizer = Adam(learningRate: valueLearningRate)
        self.gamma = gamma
        self.clipEpsilon = clipEpsilon
        self.epochs = epochs
        eval(policy, valueNetwork)
    }

    @discardableResult
    func trainStep(trajectories: [Trajectory]) -> (policyLoss: Float, valueLoss: Float) {
        var flatObservations: [Float] = []
        var flatActions: [Float] = []
        var flatOldLogProbs: [Float] = []
        var flatReturns: [Float] = []
        for trajectory in trajectories {
            let returns = discountedReturns(rewards: trajectory.rewards, gamma: gamma)
            flatReturns.append(contentsOf: returns)
            for observation in trajectory.observations {
                flatObservations.append(contentsOf: observation)
            }
            flatActions.append(contentsOf: trajectory.actions)
            flatOldLogProbs.append(contentsOf: trajectory.logProbs)
        }
        let count = flatActions.count
        let observationsArray = MLXArray(flatObservations, [count, observationDimensions])
        let actionsArray = MLXArray(flatActions, [count, 1])
        let oldLogProbsArray = MLXArray(flatOldLogProbs, [count, 1])
        let returnsArray = MLXArray(flatReturns, [count, 1])

        let valueEstimates = valueNetwork(observationsArray)
        let advantagesArray = returnsArray - valueEstimates

        func policyLoss(model: GaussianPolicy, args: (MLXArray, MLXArray, MLXArray, MLXArray)) -> [MLXArray] {
            let (observations, actions, oldLogProbs, advantages) = args
            let mean = model(observations)
            let std = exp(model.logStd)
            let newLogProbs = gaussianLogProbMLX(actions: actions, mean: mean, std: std)
            let ratio = exp(newLogProbs - oldLogProbs)
            let surrogate1 = ratio * advantages
            let surrogate2 = clip(ratio, min: 1 - self.clipEpsilon, max: 1 + self.clipEpsilon) * advantages
            return [-minimum(surrogate1, surrogate2).mean()]
        }

        func valueLoss(model: ValueNetwork, args: (MLXArray, MLXArray)) -> [MLXArray] {
            let (observations, returns) = args
            return [square(model(observations) - returns).mean()]
        }

        let policyLossAndGrad = valueAndGrad(model: policy, policyLoss)
        let valueLossAndGrad = valueAndGrad(model: valueNetwork, valueLoss)

        var lastPolicyLoss: Float = 0
        var lastValueLoss: Float = 0
        for _ in 0..<epochs {
            let (policyLosses, policyGradients) = policyLossAndGrad(
                policy, (observationsArray, actionsArray, oldLogProbsArray, advantagesArray)
            )
            policyOptimizer.update(model: policy, gradients: policyGradients)
            eval(policy, policyOptimizer)
            lastPolicyLoss = policyLosses[0].item(Float.self)

            let (valueLosses, valueGradients) = valueLossAndGrad(valueNetwork, (observationsArray, returnsArray))
            valueOptimizer.update(model: valueNetwork, gradients: valueGradients)
            eval(valueNetwork, valueOptimizer)
            lastValueLoss = valueLosses[0].item(Float.self)
        }

        return (lastPolicyLoss, lastValueLoss)
    }
}
```

- [ ] **Step 4: Run the test to verify it passes**

This exercises real MLX operations — plain `swift test` compiles it but crashes at runtime (`mlx-swift` cannot run under SwiftPM's CLI build; see Global Constraints). Use `xcodebuild test` instead:

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
xcodebuild test -scheme swift-mujoco-Package -destination 'platform=macOS'
```
Expected: `** TEST SUCCEEDED **`, with `trainStepReducesValueLossOnAFixedBatch` (and every other test in the suite) passing.

- [ ] **Step 5: Add the `ppo` case to `main.swift`**

Replace the `default:` case in `Sources/mujoco-rl-demo/main.swift` with:

```swift
case "ppo":
    let trainer = PPOTrainer(
        observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
        policyLearningRate: 3e-3, valueLearningRate: 1e-2, gamma: gamma, clipEpsilon: 0.2, epochs: 4
    )
    print("Training PPO on cartpole balance (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
    for iteration in 0..<iterations {
        let weights = trainer.policy.snapshot()
        let trajectories = await collectBatch(
            weights: weights, episodeCount: episodesPerBatch, baseSeed: UInt64(iteration) &* 1_000_003
        )
        let (policyLoss, valueLoss) = trainer.trainStep(trajectories: trajectories)
        print("iter \(iteration): mean return \(meanReturn(trajectories))  (policy loss \(policyLoss), value loss \(valueLoss))")
    }
default:
    print("Unknown algorithm '\(algorithm)'. Usage: mujoco-rl-demo [reinforce|ppo]")
```

- [ ] **Step 6: Build and run the full suite**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift build
swift test --skip MujocoRLDemoTests
xcodebuild test -scheme swift-mujoco-Package -destination 'platform=macOS'
```
Expected: `swift build` succeeds; `swift test --skip MujocoRLDemoTests` passes (MuJoCoTests, WendyMuJoCoTests, MuJoCoRLEnvTests — everything that doesn't touch MLX, which crashes under plain `swift test` per Global Constraints); `xcodebuild test` separately passes the whole scheme including `MujocoRLDemoTests`.

- [ ] **Step 7: Commit**

```bash
git add Sources/mujoco-rl-demo/PPOTrainer.swift Sources/mujoco-rl-demo/main.swift Tests/MujocoRLDemoTests/PPOTrainerTests.swift
git commit -m "feat: add PPOTrainer with clipped surrogate objective and wire up the ppo CLI path"
```

---

### Task 8: End-to-end verification, CI, and README

**Files:**
- Modify: `README.md`
- Modify: `.github/workflows/ci.yml`

**Interfaces:**
- Consumes: the complete `mujoco-rl-demo` executable (Tasks 1–7).
- Produces: nothing new — this task is verification, one CI flag, and documentation.

`mlx-swift` cannot execute at runtime under plain `swift build`/`swift run` (see Global Constraints) — running the demo for real requires building via `xcodebuild`, which bundles the `default.metallib` plain SwiftPM never produces, then executing the resulting binary directly.

- [ ] **Step 1: Build via xcodebuild and run REINFORCE end-to-end**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
xcodebuild build -scheme mujoco-rl-demo -destination 'platform=macOS' -derivedDataPath .build-xcode
.build-xcode/Build/Products/Debug/mujoco-rl-demo reinforce
```
Expected: 200 lines of `iter N: mean return X (loss Y)`. The mean return should clearly trend upward from its starting point (a random policy on this task typically averages roughly 20–40 steps before falling) over the course of the run. It does not need to hit the max of 500 on every episode by the end. If the trend is flat or noisy with no visible improvement by iteration 200, that's a hyperparameter problem, not a wiring problem — check `learningRate` and `hiddenDimensions` in `main.swift` (Task 6) before suspecting the trainer logic.

- [ ] **Step 2: Run PPO end-to-end**

```bash
.build-xcode/Build/Products/Debug/mujoco-rl-demo ppo
```
(No need to rebuild — the binary handles both subcommands.) Expected: same shape of output with `policy loss`/`value loss` instead of `loss`; mean return should trend upward, and `value loss` should trend downward (or at least not diverge) as the baseline fits the returns.

Clean up the scratch build directory once both runs are confirmed:
```bash
rm -rf .build-xcode
```

- [ ] **Step 3: Add `--skip MujocoRLDemoTests` to CI's macOS job**

In `.github/workflows/ci.yml`, find the `macos` job's `Build and test` step (the one running `swift build -v` / `swift test -v`) and change the test line:

```yaml
      - name: Build and test
        env:
          PKG_CONFIG_PATH: /Users/runner/.local/lib/pkgconfig
        run: |
          swift build -v
          swift test -v --skip MujocoRLDemoTests
```

This is the one CI change this plan requires (see Global Constraints) — without it, `swift test -v` would crash on this job the same way it does locally, since `MujocoRLDemoTests` exercises real MLX operations that plain SwiftPM-built binaries can't execute. `MujocoRLDemoTests` is verified locally via `xcodebuild test` (Tasks 5–7's own verification steps), not in CI.

- [ ] **Step 4: Update `README.md`**

Add a new bullet to the existing `## What's here` list (after the `WendyMuJoCo` bullet):

```markdown
- `mujoco-rl-demo` — REINFORCE and PPO trained against a cartpole balance
  task using [MLX-Swift](https://github.com/ml-explore/mlx-swift). macOS
  only (MLX is Metal-backed); the target and its `mlx-swift` dependency are
  gated behind `#if os(macOS)` in `Package.swift` so Linux CI never sees
  them.
```

Add a new section after the `## CI` section (or before it — match whatever reads better given the file's current section order at edit time):

```markdown
## RL sample (MLX-Swift)

`mlx-swift` can't execute at runtime under plain `swift build`/`swift run`
— SwiftPM's command-line build can't compile the Metal shaders it needs
(see [mlx-swift's README](https://github.com/ml-explore/mlx-swift#readme)).
Build and run it via `xcodebuild` instead:

    xcodebuild build -scheme mujoco-rl-demo -destination 'platform=macOS' -derivedDataPath .build-xcode
    .build-xcode/Build/Products/Debug/mujoco-rl-demo reinforce   # or: ppo

Trains a Gaussian policy to balance a cartpole. Rollout collection runs
`CartpoleEnv` episodes in parallel across a `TaskGroup`, each worker with
its own `MjModel`/`MjData` pair (`MjModel`/`MjData` are not `Sendable` and
must never be shared across threads). MLX is only used for the actual
gradient step — action sampling during rollout is a hand-rolled Swift
forward pass over a plain snapshot of the policy weights, in
`Sources/MuJoCoRLEnv/`.

`MujocoRLDemoTests` (the tests covering the MLX-dependent pieces) can't run
under plain `swift test` for the same reason — verify them locally with
`xcodebuild test -scheme swift-mujoco-Package -destination 'platform=macOS'`.
CI runs `swift test --skip MujocoRLDemoTests` and never executes this
target's tests.
```

- [ ] **Step 5: Commit**

```bash
git add README.md .github/workflows/ci.yml
git commit -m "docs: document the mujoco-rl-demo MLX-Swift RL sample; skip its tests in CI"
```

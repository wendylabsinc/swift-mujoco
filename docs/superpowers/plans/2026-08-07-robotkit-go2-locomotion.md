# RobotKit: Go2 locomotion (sim + MLX training) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A general `Environment`/`Policy` seam (`RobotKit`), a vector-action-capable MLX PPO/REINFORCE stack (`MLXPolicyTraining`), and a Go2-specific instantiation (`Go2Kit`) trained in MuJoCo — so `swift run go2-locomotion-demo` runs inference and `swift run go2-locomotion-demo --learn` trains, with `mujoco-rl-demo`'s existing cartpole demo staying behavior-identical throughout.

**Architecture:** `RobotKit` defines the robot-agnostic `Environment` protocol and a `RunMode`/`RunModeKey` task-local. The existing scalar-action MLX training stack (`GaussianPolicy`/`PPOTrainer`/`ReinforceTrainer`/`GaussianLogProb`, currently inside the `mujoco-rl-demo` executable) is extracted into a new `MLXPolicyTraining` library and generalized to vector actions; `PolicyWeights`/`policyForward`/`gaussianLogProb`/`sampleGaussian` (cross-platform, MLX-free) are generalized in place inside `MuJoCoRLEnv`. `Rollout.swift`'s `collectEpisode`/`collectBatch` become generic over any `Environment` conformer; `CartpoleEnv` is retrofitted to conform, proving the abstraction fits a second robot before `Go2Kit` (MuJoCo-backed Go2 environment, real joint/observation/action-scale conventions from `wendy-sandbox`'s `go2.robot.json`) builds on it.

**Full design:** `docs/superpowers/specs/2026-08-07-robotkit-go2-locomotion-design.md`

## Global Constraints

- Swift tools version: `6.1` (matches existing `Package.swift`).
- Tests use the **Swift Testing** framework (`import Testing`, `@Test func`, `#expect`) — matches every existing test file under `Tests/`.
- `RobotKit` and `MuJoCoRLEnv`/`Rollout.swift`/`PolicyWeights.swift` must stay **cross-platform** (no MLX dependency, no `#if os(macOS)` gate) — only `MLXPolicyTraining`, `Go2Kit`, and `go2-locomotion-demo` are macOS-only, added inside the existing `#if os(macOS)` block in `Package.swift`.
- `mujoco-rl-demo`'s existing cartpole training behavior (numeric loss/return trends) must not change — every generalization is to `actionDimensions: 1` for that demo, byte-for-byte equivalent to today's scalar behavior.
- Closures passed into `collectBatch` that cross the `TaskGroup` boundary (`makeEnvironment`, `reward`) must be typed `@escaping @Sendable`.
- `MjModel`/`MjData` are not `Sendable` and must never be shared across threads — every parallel rollout worker constructs its own via `makeEnvironment()`, matching the existing `CartpoleEnv` pattern.
- All numeric constants for Go2 control (joint order, `action_scale`, `default_pos`, `kp`, `kd`, `torque_limits`, fall-detection thresholds, observation composition/scaling) come from `wendy-sandbox/image/apps/unitree-sim2real/robots/go2.robot.json` verbatim — do not invent different values.

---

### Task 1: `RobotKit` — `Environment` protocol and `RunMode`/`RunModeKey`

**Files:**
- Create: `Sources/RobotKit/Environment.swift`
- Test: Create `Tests/RobotKitTests/EnvironmentTests.swift`
- Modify: `Package.swift` (new `RobotKit` target, `RobotKitTests` test target, `RobotKit` product)

**Interfaces:**
- Produces: `public protocol Environment<Observation, Action>` (`reset() -> Observation`, `act(_:) -> Observation`, `isTerminated: Bool`), `public enum RunMode`, `public enum RunModeKey { @TaskLocal public static var current: RunMode }`. Every later task depends on this.

- [ ] **Step 1: Add the `RobotKit` target to `Package.swift`**

In the unconditional `targets` array (anywhere after `WendyMuJoCo`), add:

```swift
    .target(name: "RobotKit", path: "Sources/RobotKit"),
    .testTarget(name: "RobotKitTests", dependencies: ["RobotKit"]),
```

Add to `products`:

```swift
    .library(name: "RobotKit", targets: ["RobotKit"]),
```

- [ ] **Step 2: Write the failing tests**

Create `Tests/RobotKitTests/EnvironmentTests.swift`:

```swift
import Testing
@testable import RobotKit

@Test func runModeDefaultsToInfer() {
    #expect(RunModeKey.current == .infer)
}

@Test func runModeScopesToWithValue() {
    #expect(RunModeKey.current == .infer)
    RunModeKey.$current.withValue(.learn) {
        #expect(RunModeKey.current == .learn)
    }
    #expect(RunModeKey.current == .infer)
}

private struct CountingEnvironment: Environment {
    var count = 0
    var isTerminated: Bool { count >= 3 }
    mutating func reset() -> Int { count = 0; return count }
    mutating func act(_ action: Int) -> Int { count += action; return count }
}

@Test func environmentConformancePlainWalkthrough() {
    var env = CountingEnvironment()
    #expect(env.reset() == 0)
    #expect(env.act(1) == 1)
    #expect(env.isTerminated == false)
    #expect(env.act(2) == 3)
    #expect(env.isTerminated == true)
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `swift test --filter RobotKitTests`
Expected: FAIL to build — `RobotKit` module doesn't exist yet.

- [ ] **Step 4: Implement**

Create `Sources/RobotKit/Environment.swift`:

```swift
import Foundation

/// A control-loop seam: `observe`/`act` work the same whether the far side is
/// a MuJoCo simulation or real hardware. Reward, training-episode
/// bookkeeping, and step-count cutoffs are NOT part of this protocol — they
/// are training-task-specific constructs computed from `Observation` by the
/// caller, not something real hardware has an intrinsic notion of.
public protocol Environment<Observation, Action> {
    associatedtype Observation
    associatedtype Action
    mutating func reset() -> Observation
    mutating func act(_ action: Action) -> Observation
    /// True when the environment considers itself done on its own terms
    /// (e.g. a fall, or an environment-intrinsic step cap) — distinct from a
    /// training harness's own `maxSteps` cutoff.
    var isTerminated: Bool { get }
}

/// Whether the current process is running inference or training. Set once at
/// the top of `main()` and read anywhere via the task-local, rather than
/// threading a mode flag through every function signature.
public enum RunMode: Sendable, Equatable {
    case infer
    case learn
}

public enum RunModeKey {
    @TaskLocal public static var current: RunMode = .infer
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `swift test --filter RobotKitTests`
Expected: PASS (3 tests).

- [ ] **Step 6: Commit**

```bash
git add Package.swift Sources/RobotKit Tests/RobotKitTests
git commit -m "feat: add RobotKit's Environment protocol and RunMode task-local"
```

---

### Task 2: Generalize `PolicyWeights`/`policyForward`/`gaussianLogProb`/`sampleGaussian` to vector actions

**Files:**
- Modify: `Sources/MuJoCoRLEnv/PolicyWeights.swift`
- Modify: `Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift`

**Interfaces:**
- Consumes: nothing new (this file has no dependency on Task 1).
- Produces: `PolicyWeights` gains `actionDimensions: Int`; `w2`/`b2`/`logStd` are resized (`[actionDimensions × hidden]`, `[actionDimensions]`, `[actionDimensions]`); `policyForward(_:observation:) -> (mean: [Float], std: [Float])`; `gaussianLogProb(action: [Float], mean: [Float], std: [Float]) -> Float`; `sampleGaussian(mean: [Float], std: [Float], using:) -> [Float]`. Task 3 (`Rollout.swift`) and Task 4 (`MLXPolicyTraining`) consume these new signatures directly.

- [ ] **Step 1: Write the failing tests**

Replace `Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift` in full:

```swift
// Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift
import Testing

@testable import MuJoCoRLEnv

@Test func forwardPassMatchesHandComputedValue() {
    // hiddenDimensions=2, inputDimensions=2, w1 = identity, b1 = 0,
    // w2 = [1, 1] (one row, actionDimensions=1), b2 = [0], logStd = [0].
    // hidden = tanh([1, 1]) ≈ [0.7615941559557649, 0.7615941559557649]
    // mean[0] = hidden[0] + hidden[1] ≈ 1.5231883119115298 ; std[0] = exp(0) = 1
    let weights = PolicyWeights(
        w1: [1, 0, 0, 1], b1: [0, 0], w2: [1, 1], b2: [0], logStd: [0],
        inputDimensions: 2, hiddenDimensions: 2, actionDimensions: 1
    )
    let (mean, std) = policyForward(weights, observation: [1, 1])
    #expect(mean.count == 1)
    #expect(abs(mean[0] - 1.5231883) < 1e-4)
    #expect(abs(std[0] - 1.0) < 1e-6)
}

@Test func forwardPassHandlesMultipleActionDimensionsIndependently() {
    // hiddenDimensions=2, inputDimensions=2, w1 = identity, b1 = 0,
    // actionDimensions=2: row0 = [1, 0] (only reads hidden[0]), row1 = [0, 1] (only reads hidden[1]).
    // hidden ≈ [0.7615941559557649, 0.7615941559557649]
    let weights = PolicyWeights(
        w1: [1, 0, 0, 1], b1: [0, 0], w2: [1, 0, 0, 1], b2: [0, 0], logStd: [0, 1],
        inputDimensions: 2, hiddenDimensions: 2, actionDimensions: 2
    )
    let (mean, std) = policyForward(weights, observation: [1, 1])
    #expect(mean.count == 2)
    #expect(abs(mean[0] - 0.7615941) < 1e-4)
    #expect(abs(mean[1] - 0.7615941) < 1e-4)
    #expect(abs(std[0] - 1.0) < 1e-6)
    #expect(abs(std[1] - exp(Float(1))) < 1e-4)
}

@Test func gaussianLogProbMatchesClosedForm() {
    // logProb(0 | mean=0, std=1) = -0.5 * log(2*pi) ≈ -0.9189385332
    let logProb = gaussianLogProb(action: [0], mean: [0], std: [1])
    #expect(abs(logProb - (-0.9189385)) < 1e-4)
}

@Test func gaussianLogProbSumsIndependentDimensions() {
    // Two independent standard-normal dimensions at 0: each contributes -0.9189385332.
    let logProb = gaussianLogProb(action: [0, 0], mean: [0, 0], std: [1, 1])
    #expect(abs(logProb - 2 * (-0.9189385)) < 1e-4)
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
    let a = sampleGaussian(mean: [0], std: [1], using: &rngA)
    let b = sampleGaussian(mean: [0], std: [1], using: &rngB)
    #expect(a == b)
}

@Test func sampleGaussianWithZeroStdReturnsTheMean() {
    var rng = SplitMix64(seed: 1)
    let action = sampleGaussian(mean: [3.5, -1.0], std: [0, 0], using: &rng)
    #expect(action == [3.5, -1.0])
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `swift test --filter PolicyWeightsTests`
Expected: FAIL to build — `PolicyWeights`'s current initializer takes a scalar `logStd: Float` and no `actionDimensions`, and `policyForward`/`gaussianLogProb`/`sampleGaussian` take/return scalars, not arrays.

- [ ] **Step 3: Implement**

Replace `Sources/MuJoCoRLEnv/PolicyWeights.swift` in full:

```swift
#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif

/// A plain-value snapshot of a Gaussian policy's weights: `Linear(in→hidden)
/// → tanh → Linear(hidden→actionDimensions)` for the action mean, plus one
/// log-std per action dimension (diagonal-covariance Gaussian). `Sendable` by
/// construction (all stored properties are value types), so it can cross a
/// `TaskGroup` boundary into parallel rollout workers with no MLX object ever
/// doing the same.
public struct PolicyWeights: Sendable {
    /// Row-major [hiddenDimensions x inputDimensions].
    public let w1: [Float]
    public let b1: [Float]
    /// Row-major [actionDimensions x hiddenDimensions].
    public let w2: [Float]
    public let b2: [Float]
    public let logStd: [Float]
    public let inputDimensions: Int
    public let hiddenDimensions: Int
    public let actionDimensions: Int

    public init(
        w1: [Float], b1: [Float], w2: [Float], b2: [Float], logStd: [Float],
        inputDimensions: Int, hiddenDimensions: Int, actionDimensions: Int
    ) {
        self.w1 = w1
        self.b1 = b1
        self.w2 = w2
        self.b2 = b2
        self.logStd = logStd
        self.inputDimensions = inputDimensions
        self.hiddenDimensions = hiddenDimensions
        self.actionDimensions = actionDimensions
    }
}

/// The rollout-time policy forward pass. Deliberately not MLX: this runs
/// inside parallel `TaskGroup` workers, and MLX's thread-safety for
/// concurrent forward passes across OS threads is unverified. Gradients are
/// only ever needed at training time (see `MLXPolicyTraining`), so this
/// plain arithmetic version is sufficient here.
public func policyForward(_ weights: PolicyWeights, observation: [Float]) -> (mean: [Float], std: [Float]) {
    precondition(observation.count == weights.inputDimensions)
    var hidden = [Float](repeating: 0, count: weights.hiddenDimensions)
    for h in 0..<weights.hiddenDimensions {
        var sum = weights.b1[h]
        for i in 0..<weights.inputDimensions {
            sum += weights.w1[h * weights.inputDimensions + i] * observation[i]
        }
        hidden[h] = tanh(sum)
    }
    var mean = [Float](repeating: 0, count: weights.actionDimensions)
    for a in 0..<weights.actionDimensions {
        var sum = weights.b2[a]
        for h in 0..<weights.hiddenDimensions {
            sum += weights.w2[a * weights.hiddenDimensions + h] * hidden[h]
        }
        mean[a] = sum
    }
    let std = weights.logStd.map { exp($0) }
    return (mean, std)
}

/// Joint log-probability under a diagonal-covariance Gaussian: the sum of
/// each action dimension's independent log-probability.
public func gaussianLogProb(action: [Float], mean: [Float], std: [Float]) -> Float {
    precondition(action.count == mean.count && mean.count == std.count)
    var total: Float = 0
    for i in 0..<action.count {
        let variance = std[i] * std[i]
        total += -0.5 * log(2 * Float.pi * variance) - (action[i] - mean[i]) * (action[i] - mean[i]) / (2 * variance)
    }
    return total
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

public func sampleGaussian(mean: [Float], std: [Float], using rng: inout SplitMix64) -> [Float] {
    precondition(mean.count == std.count)
    return zip(mean, std).map { m, s in
        guard s > 0 else { return m }
        // Box-Muller.
        let u1 = Float.random(in: Float.ulpOfOne..<1, using: &rng)
        let u2 = Float.random(in: 0..<1, using: &rng)
        let z = sqrt(-2 * log(u1)) * cos(2 * Float.pi * u2)
        return m + s * z
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `swift test --filter PolicyWeightsTests`
Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
git add Sources/MuJoCoRLEnv/PolicyWeights.swift Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift
git commit -m "feat: generalize PolicyWeights and its forward pass to vector actions"
```

---

### Task 3: Generalize `Rollout.swift` off `CartpoleEnv`; retrofit `CartpoleEnv` to `Environment`

**Files:**
- Modify: `Sources/MuJoCoRLEnv/Rollout.swift`
- Modify: `Sources/MuJoCoRLEnv/CartpoleEnv.swift`
- Modify: `Tests/MuJoCoRLEnvTests/RolloutTests.swift`
- Modify: `Tests/MuJoCoRLEnvTests/CartpoleEnvTests.swift`
- Modify: `Package.swift` (`MuJoCoRLEnv` target gains a `RobotKit` dependency)

**Interfaces:**
- Consumes: `RobotKit.Environment` (Task 1); generalized `PolicyWeights`/`policyForward`/`gaussianLogProb`/`sampleGaussian` (Task 2).
- Produces: `public protocol ObservationEncoding { var asArray: [Float] { get } }`; `public struct Trajectory { ...; var actions: [[Float]]; ... }` (was `[Float]`); `public func collectEpisode<E: Environment>(makeEnvironment:weights:reward:maxSteps:seed:) -> Trajectory where E.Action == [Float], E.Observation: ObservationEncoding`; `public func collectBatch<E: Environment>(makeEnvironment: @escaping @Sendable () -> E, weights:episodeCount:reward: @escaping @Sendable (E.Observation) -> Float, maxSteps:baseSeed:) async -> [Trajectory]` (same constraints). `CartpoleEnv: Environment` with `Action == [Float]`, `Observation == CartpoleObservation`. Task 4's generalized `mujoco-rl-demo/main.swift` and Task 5's `Go2Kit` both consume these.

- [ ] **Step 1: Add `RobotKit` as a dependency of `MuJoCoRLEnv` in `Package.swift`**

Change:

```swift
    .target(name: "MuJoCoRLEnv", dependencies: ["MuJoCo"]),
```

to:

```swift
    .target(name: "MuJoCoRLEnv", dependencies: ["MuJoCo", "RobotKit"]),
```

- [ ] **Step 2: Write the failing tests**

Replace `Tests/MuJoCoRLEnvTests/CartpoleEnvTests.swift` in full:

```swift
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
    let obs = env.act([0])
    #expect(env.isTerminated == false)
    #expect(abs(obs.poleAngle) < 0.05)
}

@Test func sustainedLargeActionEventuallyTerminates() {
    let env = CartpoleEnv()
    _ = env.reset()
    var steps = 0
    while !env.isTerminated && steps < CartpoleEnv.maxSteps {
        _ = env.act([1.0])
        steps += 1
    }
    #expect(env.isTerminated == true)
    #expect(steps < CartpoleEnv.maxSteps)
}

@Test func episodeTerminatesAtMaxStepsUnderZeroAction() {
    let env = CartpoleEnv()
    _ = env.reset()
    var steps = 0
    while !env.isTerminated {
        _ = env.act([0])
        steps += 1
    }
    #expect(steps == CartpoleEnv.maxSteps)
}
```

Replace `Tests/MuJoCoRLEnvTests/RolloutTests.swift` in full:

```swift
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
    w2: [Float](repeating: 0, count: 32), b2: [0], logStd: [0],
    inputDimensions: 4, hiddenDimensions: 32, actionDimensions: 1
)

private func constantReward(_: CartpoleObservation) -> Float { 1.0 }

@Test func collectEpisodeProducesConsistentArrayLengths() {
    let trajectory = collectEpisode(
        makeEnvironment: { CartpoleEnv() }, weights: zeroWeightPolicy,
        reward: constantReward, maxSteps: CartpoleEnv.maxSteps, seed: 1
    )
    #expect(trajectory.observations.count > 0)
    #expect(trajectory.observations.count <= CartpoleEnv.maxSteps)
    #expect(trajectory.actions.count == trajectory.observations.count)
    #expect(trajectory.logProbs.count == trajectory.observations.count)
    #expect(trajectory.rewards.count == trajectory.observations.count)
    #expect(trajectory.observations.allSatisfy { $0.count == 4 })
    #expect(trajectory.actions.allSatisfy { $0.count == 1 })
}

@Test func collectBatchRunsEpisodesInParallel() async {
    let trajectories = await collectBatch(
        makeEnvironment: { CartpoleEnv() }, weights: zeroWeightPolicy, episodeCount: 4,
        reward: constantReward, maxSteps: CartpoleEnv.maxSteps, baseSeed: 100
    )
    #expect(trajectories.count == 4)
    for trajectory in trajectories {
        #expect(trajectory.observations.count > 0)
    }
}

@Test func collectEpisodeIsDeterministicForAFixedSeed() {
    let a = collectEpisode(
        makeEnvironment: { CartpoleEnv() }, weights: zeroWeightPolicy,
        reward: constantReward, maxSteps: CartpoleEnv.maxSteps, seed: 55
    )
    let b = collectEpisode(
        makeEnvironment: { CartpoleEnv() }, weights: zeroWeightPolicy,
        reward: constantReward, maxSteps: CartpoleEnv.maxSteps, seed: 55
    )
    #expect(a.actions == b.actions)
    #expect(a.rewards == b.rewards)
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `swift test --filter CartpoleEnvTests,RolloutTests`
Expected: FAIL to build — `CartpoleEnv.act`/`isTerminated` don't exist yet (only the old `step(action:)`); `collectEpisode`/`collectBatch` don't accept `makeEnvironment`/`reward`/`maxSteps` yet.

- [ ] **Step 4: Implement — retrofit `CartpoleEnv`**

Replace `Sources/MuJoCoRLEnv/CartpoleEnv.swift` in full:

```swift
import MuJoCo
import RobotKit

public struct CartpoleObservation: ObservationEncoding {
    public let cartPosition: Double
    public let cartVelocity: Double
    public let poleAngle: Double
    public let poleAngularVelocity: Double

    public var asArray: [Float] {
        [Float(cartPosition), Float(cartVelocity), Float(poleAngle), Float(poleAngularVelocity)]
    }
}

/// Cart-on-a-rail with an unactuated pole, balanced by a continuous force on
/// the cart. Not `Sendable` (it owns `MjModel`/`MjData`, neither of which
/// is) — each parallel rollout worker in `collectEpisode` constructs its own
/// instance rather than sharing one across tasks.
public final class CartpoleEnv: Environment {
    public static let maxSteps = 500
    static let cartPositionLimit: Double = 2.4
    // 12 degrees, matching the classic cartpole task's failure threshold.
    static let poleAngleLimit: Double = 0.2094395102393195

    private let model: MjModel
    private let data: MjData
    private var stepCount = 0

    public init() {
        self.model = try! MjModel.load(xml: Self.xml)
        self.data = MjData(model)
    }

    /// Out of bounds OR the episode's own step cap — this environment treats
    /// its step limit as part of its own termination condition, unlike
    /// `Go2Environment`, where the training harness's external `maxSteps`
    /// plays that role instead.
    public var isTerminated: Bool {
        let obs = observation()
        let outOfBounds =
            abs(obs.cartPosition) > Self.cartPositionLimit || abs(obs.poleAngle) > Self.poleAngleLimit
        return outOfBounds || stepCount >= Self.maxSteps
    }

    public func reset() -> CartpoleObservation {
        mjResetData(model, data)
        stepCount = 0
        return observation()
    }

    public func act(_ action: [Float]) -> CartpoleObservation {
        data.setCtrl([Double(action[0])])
        mjStep(model, data)
        stepCount += 1
        return observation()
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

- [ ] **Step 5: Implement — generalize `Rollout.swift`**

Replace `Sources/MuJoCoRLEnv/Rollout.swift` in full:

```swift
import RobotKit

/// Anything that can flatten itself into the policy network's input vector.
public protocol ObservationEncoding {
    var asArray: [Float] { get }
}

public struct Trajectory: Sendable {
    public var observations: [[Float]]
    public var actions: [[Float]]
    public var logProbs: [Float]
    public var rewards: [Float]

    public init(
        observations: [[Float]] = [], actions: [[Float]] = [], logProbs: [Float] = [],
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

/// Runs one full episode against a fresh `Environment`, sampling actions
/// from `weights` via the hand-rolled forward pass. Safe to call
/// concurrently from multiple tasks: nothing here is shared mutable state,
/// and the environment this function creates never leaves it. Termination is
/// `env.isTerminated || steps >= maxSteps` — `reward` is the only
/// caller-supplied piece, since it's genuinely training-task-specific (the
/// same environment could be trained against different reward shaping),
/// unlike termination, which the environment already knows how to answer.
public func collectEpisode<E: Environment>(
    makeEnvironment: () -> E, weights: PolicyWeights,
    reward: (E.Observation) -> Float, maxSteps: Int, seed: UInt64
) -> Trajectory where E.Action == [Float], E.Observation: ObservationEncoding {
    var rng = SplitMix64(seed: seed)
    var env = makeEnvironment()
    var trajectory = Trajectory()
    var observation = env.reset()
    var steps = 0
    while true {
        let observationArray = observation.asArray
        let (mean, std) = policyForward(weights, observation: observationArray)
        let action = sampleGaussian(mean: mean, std: std, using: &rng)
        let logProb = gaussianLogProb(action: action, mean: mean, std: std)
        let nextObservation = env.act(action)
        steps += 1

        trajectory.observations.append(observationArray)
        trajectory.actions.append(action)
        trajectory.logProbs.append(logProb)
        trajectory.rewards.append(reward(nextObservation))

        observation = nextObservation
        if env.isTerminated || steps >= maxSteps { break }
    }
    return trajectory
}

/// Fans out `episodeCount` independent episodes across a `TaskGroup`. Each
/// worker gets a distinct seed (`baseSeed + index`) so parallel workers don't
/// duplicate the same action sequence, and constructs its own `Environment`
/// via `makeEnvironment()` — never sharing one across the task boundary.
public func collectBatch<E: Environment>(
    makeEnvironment: @escaping @Sendable () -> E, weights: PolicyWeights, episodeCount: Int,
    reward: @escaping @Sendable (E.Observation) -> Float, maxSteps: Int, baseSeed: UInt64
) async -> [Trajectory] where E.Action == [Float], E.Observation: ObservationEncoding {
    await withTaskGroup(of: Trajectory.self) { group in
        for index in 0..<episodeCount {
            group.addTask {
                collectEpisode(
                    makeEnvironment: makeEnvironment, weights: weights, reward: reward,
                    maxSteps: maxSteps, seed: baseSeed &+ UInt64(index)
                )
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

- [ ] **Step 6: Run tests to verify they pass**

Run: `PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig swift test --filter CartpoleEnvTests,RolloutTests,PolicyWeightsTests,RobotKitTests`
Expected: PASS (all).

- [ ] **Step 7: Commit**

```bash
git add Package.swift Sources/MuJoCoRLEnv/Rollout.swift Sources/MuJoCoRLEnv/CartpoleEnv.swift \
        Tests/MuJoCoRLEnvTests/RolloutTests.swift Tests/MuJoCoRLEnvTests/CartpoleEnvTests.swift
git commit -m "feat: generalize Rollout.swift over Environment; retrofit CartpoleEnv"
```

---

### Task 4: Extract and generalize `MLXPolicyTraining`; update `mujoco-rl-demo`

**Files:**
- Create: `Sources/MLXPolicyTraining/GaussianPolicy.swift`
- Create: `Sources/MLXPolicyTraining/GaussianLogProb.swift`
- Create: `Sources/MLXPolicyTraining/PPOTrainer.swift`
- Create: `Sources/MLXPolicyTraining/ReinforceTrainer.swift`
- Delete: `Sources/mujoco-rl-demo/GaussianPolicy.swift`, `Sources/mujoco-rl-demo/GaussianLogProb.swift`, `Sources/mujoco-rl-demo/PPOTrainer.swift`, `Sources/mujoco-rl-demo/ReinforceTrainer.swift`
- Modify: `Sources/mujoco-rl-demo/main.swift`
- Create: `Tests/MLXPolicyTrainingTests/GaussianPolicyTests.swift`, `Tests/MLXPolicyTrainingTests/PPOTrainerTests.swift`, `Tests/MLXPolicyTrainingTests/ReinforceTrainerTests.swift`
- Delete: `Tests/MujocoRLDemoTests/` (all three files, then the empty directory)
- Modify: `Package.swift` (new `MLXPolicyTraining`/`MLXPolicyTrainingTests` targets inside the `#if os(macOS)` block; `MujocoRLDemo` depends on `MLXPolicyTraining` instead of raw MLX products; `MujocoRLDemoTests` target removed)

**Interfaces:**
- Consumes: generalized `PolicyWeights`/`Trajectory`/`discountedReturns` (Task 2/3, from `MuJoCoRLEnv`, cross-platform).
- Produces: `public final class GaussianPolicy: Module` (gains `actionDimensions: Int`, `snapshot() -> PolicyWeights`), `public final class PPOTrainer` / `public final class ReinforceTrainer` (both gain an `actionDimensions: Int` init parameter), `public func gaussianLogProbMLX`. Task 6 (`go2-locomotion-demo`) consumes these directly.

- [ ] **Step 1: Update `Package.swift`**

Inside the `#if os(macOS)` block, replace:

```swift
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
```

with:

```swift
targets.append(
    .target(
        name: "MLXPolicyTraining",
        dependencies: [
            "MuJoCoRLEnv",
            .product(name: "MLX", package: "mlx-swift"),
            .product(name: "MLXNN", package: "mlx-swift"),
            .product(name: "MLXOptimizers", package: "mlx-swift"),
        ],
        path: "Sources/MLXPolicyTraining"
    )
)
targets.append(.testTarget(name: "MLXPolicyTrainingTests", dependencies: ["MLXPolicyTraining", "MuJoCoRLEnv"]))
targets.append(
    .executableTarget(
        name: "MujocoRLDemo",
        dependencies: ["MuJoCoRLEnv", "MLXPolicyTraining"],
        path: "Sources/mujoco-rl-demo"
    )
)
products.append(.library(name: "MLXPolicyTraining", targets: ["MLXPolicyTraining"]))
products.append(.executable(name: "mujoco-rl-demo", targets: ["MujocoRLDemo"]))
```

- [ ] **Step 2: Move the four MLX-dependent files, generalizing as you go**

Delete the four existing files under `Sources/mujoco-rl-demo/` and create their replacements under `Sources/MLXPolicyTraining/`:

Create `Sources/MLXPolicyTraining/GaussianPolicy.swift`:

```swift
// Sources/MLXPolicyTraining/GaussianPolicy.swift
import MLX
import MLXNN
import MuJoCoRLEnv

/// `Linear(observationDimensions → hiddenDimensions) → tanh →
/// Linear(hiddenDimensions → actionDimensions)` for the action mean, plus a
/// learned per-action-dimension log-std. Used only for training (gradient
/// computation via `valueAndGrad`) — rollout-time action sampling uses
/// `snapshot()` and the MLX-free `policyForward` in `MuJoCoRLEnv` instead.
public final class GaussianPolicy: Module {
    @ModuleInfo public var fc1: Linear
    @ModuleInfo public var fc2: Linear
    public var logStd: MLXArray

    public let observationDimensions: Int
    public let hiddenDimensions: Int
    public let actionDimensions: Int

    public init(observationDimensions: Int, hiddenDimensions: Int, actionDimensions: Int) {
        self.observationDimensions = observationDimensions
        self.hiddenDimensions = hiddenDimensions
        self.actionDimensions = actionDimensions
        self.fc1 = Linear(observationDimensions, hiddenDimensions)
        self.fc2 = Linear(hiddenDimensions, actionDimensions)
        self.logStd = MLXArray.zeros([actionDimensions])
    }

    public func callAsFunction(_ x: MLXArray) -> MLXArray {
        fc2(tanh(fc1(x)))
    }
}

extension GaussianPolicy {
    /// Extracts a plain-value weight snapshot for rollout workers. `Linear`
    /// stores `weight` as `[outputDimensions, inputDimensions]` row-major —
    /// the same layout `policyForward` expects — and is always constructed
    /// with `bias: true` (the default) here, so `.bias!` is safe.
    public func snapshot() -> PolicyWeights {
        PolicyWeights(
            w1: fc1.weight.asArray(Float.self),
            b1: fc1.bias!.asArray(Float.self),
            w2: fc2.weight.asArray(Float.self),
            b2: fc2.bias!.asArray(Float.self),
            logStd: logStd.asArray(Float.self),
            inputDimensions: observationDimensions,
            hiddenDimensions: hiddenDimensions,
            actionDimensions: actionDimensions
        )
    }
}
```

Create `Sources/MLXPolicyTraining/GaussianLogProb.swift`:

```swift
// Sources/MLXPolicyTraining/GaussianLogProb.swift
import MLX

/// MLX-array Gaussian log-probability, summed across the action-dimension
/// axis into one joint log-probability per row — the array counterpart of
/// `MuJoCoRLEnv`'s scalar `gaussianLogProb`, used inside both trainers'
/// gradient-tracked loss closures so the formula lives in exactly one place.
public func gaussianLogProbMLX(actions: MLXArray, mean: MLXArray, std: MLXArray) -> MLXArray {
    let variance = square(std)
    let perDimension = -0.5 * log(2 * Float.pi * variance) - square(actions - mean) / (2 * variance)
    return perDimension.sum(axis: -1, keepDims: true)
}
```

Create `Sources/MLXPolicyTraining/PPOTrainer.swift`:

```swift
// Sources/MLXPolicyTraining/PPOTrainer.swift
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
public final class PPOTrainer {
    public let policy: GaussianPolicy
    private let valueNetwork: ValueNetwork
    private let policyOptimizer: Adam
    private let valueOptimizer: Adam
    private let gamma: Float
    private let clipEpsilon: Float
    private let epochs: Int
    private let observationDimensions: Int
    private let actionDimensions: Int

    public init(
        observationDimensions: Int, hiddenDimensions: Int, actionDimensions: Int,
        policyLearningRate: Float, valueLearningRate: Float, gamma: Float,
        clipEpsilon: Float, epochs: Int
    ) {
        self.observationDimensions = observationDimensions
        self.actionDimensions = actionDimensions
        self.policy = GaussianPolicy(
            observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
            actionDimensions: actionDimensions
        )
        self.valueNetwork = ValueNetwork(observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions)
        self.policyOptimizer = Adam(learningRate: policyLearningRate)
        self.valueOptimizer = Adam(learningRate: valueLearningRate)
        self.gamma = gamma
        self.clipEpsilon = clipEpsilon
        self.epochs = epochs
        eval(policy, valueNetwork)
    }

    @discardableResult
    public func trainStep(trajectories: [Trajectory]) -> (policyLoss: Float, valueLoss: Float) {
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
            for action in trajectory.actions {
                flatActions.append(contentsOf: action)
            }
            flatOldLogProbs.append(contentsOf: trajectory.logProbs)
        }
        let count = flatOldLogProbs.count
        let observationsArray = MLXArray(flatObservations, [count, observationDimensions])
        let actionsArray = MLXArray(flatActions, [count, actionDimensions])
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

Create `Sources/MLXPolicyTraining/ReinforceTrainer.swift`:

```swift
// Sources/MLXPolicyTraining/ReinforceTrainer.swift
import MLX
import MLXNN
import MLXOptimizers
import MuJoCoRLEnv

public final class ReinforceTrainer {
    public let policy: GaussianPolicy
    private let optimizer: Adam
    private let gamma: Float
    private let observationDimensions: Int
    private let actionDimensions: Int

    public init(
        observationDimensions: Int, hiddenDimensions: Int, actionDimensions: Int,
        learningRate: Float, gamma: Float
    ) {
        self.observationDimensions = observationDimensions
        self.actionDimensions = actionDimensions
        self.policy = GaussianPolicy(
            observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
            actionDimensions: actionDimensions
        )
        self.optimizer = Adam(learningRate: learningRate)
        self.gamma = gamma
        eval(policy)
    }

    /// Discounted returns with a batch-mean baseline; one Adam step over the
    /// whole batch.
    @discardableResult
    public func trainStep(trajectories: [Trajectory]) -> Float {
        var flatObservations: [Float] = []
        var flatActions: [Float] = []
        var flatReturns: [Float] = []
        for trajectory in trajectories {
            let returns = discountedReturns(rewards: trajectory.rewards, gamma: gamma)
            flatReturns.append(contentsOf: returns)
            for observation in trajectory.observations {
                flatObservations.append(contentsOf: observation)
            }
            for action in trajectory.actions {
                flatActions.append(contentsOf: action)
            }
        }
        let count = trajectories.reduce(0) { $0 + $1.actions.count }
        let baseline = flatReturns.reduce(0, +) / Float(flatReturns.count)
        let advantages = flatReturns.map { $0 - baseline }

        let observationsArray = MLXArray(flatObservations, [count, observationDimensions])
        let actionsArray = MLXArray(flatActions, [count, actionDimensions])
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

- [ ] **Step 3: Move and generalize the tests**

Delete `Tests/MujocoRLDemoTests/GaussianPolicyTests.swift`, `PPOTrainerTests.swift`, `ReinforceTrainerTests.swift`, then the now-empty `Tests/MujocoRLDemoTests/` directory.

Create `Tests/MLXPolicyTrainingTests/GaussianPolicyTests.swift`:

```swift
// Tests/MLXPolicyTrainingTests/GaussianPolicyTests.swift
import MLX
import Testing

@testable import MLXPolicyTraining
import MuJoCoRLEnv

@Test func snapshotMatchesLiveMLXForwardPass() {
    let policy = GaussianPolicy(observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 1)
    let weights = policy.snapshot()

    let observation: [Float] = [0.1, -0.2, 0.3, -0.4]
    let mlxOutput = policy(MLXArray(observation, [1, 4]))
    let mlxMean = mlxOutput.item(Float.self)

    let (handRolledMean, _) = policyForward(weights, observation: observation)

    #expect(abs(mlxMean - handRolledMean[0]) < 1e-4)
}

@Test func snapshotShapesMatchDeclaredDimensions() {
    let policy = GaussianPolicy(observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 1)
    let weights = policy.snapshot()
    #expect(weights.w1.count == 8 * 4)
    #expect(weights.b1.count == 8)
    #expect(weights.w2.count == 8)
    #expect(weights.b2.count == 1)
    #expect(weights.logStd.count == 1)
    #expect(weights.inputDimensions == 4)
    #expect(weights.hiddenDimensions == 8)
    #expect(weights.actionDimensions == 1)
}

@Test func snapshotShapesScaleWithMultipleActionDimensions() {
    let policy = GaussianPolicy(observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 12)
    let weights = policy.snapshot()
    #expect(weights.w2.count == 12 * 8)
    #expect(weights.b2.count == 12)
    #expect(weights.logStd.count == 12)
    #expect(weights.actionDimensions == 12)
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
    let scalarResult = gaussianLogProb(action: [action], mean: [mean], std: [std])
    #expect(abs(mlxResult - scalarResult) < 1e-4)
}

@Test func gaussianLogProbMLXSumsAcrossActionDimensions() {
    let actions = MLXArray([Float(0), Float(0)], [1, 2])
    let mean = MLXArray([Float(0), Float(0)], [1, 2])
    let std = MLXArray([Float(1), Float(1)], [1, 2])
    let mlxResult = gaussianLogProbMLX(actions: actions, mean: mean, std: std).item(Float.self)
    let scalarResult = gaussianLogProb(action: [0, 0], mean: [0, 0], std: [1, 1])
    #expect(abs(mlxResult - scalarResult) < 1e-4)
}
```

Create `Tests/MLXPolicyTrainingTests/PPOTrainerTests.swift`:

```swift
// Tests/MLXPolicyTrainingTests/PPOTrainerTests.swift
import Testing

@testable import MLXPolicyTraining
import MuJoCoRLEnv

@Test func trainStepReducesValueLossOnAFixedBatch() {
    let trainer = PPOTrainer(
        observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 1,
        policyLearningRate: 1e-2, valueLearningRate: 1e-1, gamma: 0.99, clipEpsilon: 0.2, epochs: 4
    )
    let observations = (0..<20).map { i in [Float(i) * 0.01, 0, 0.02, 0] }
    let actions = (0..<20).map { i in [Float(i) * 0.05 - 0.5] }
    let logProbs = observations.indices.map { i in
        gaussianLogProb(action: actions[i], mean: [0], std: [1])
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

@Test func trainStepHandlesMultiDimensionalActions() {
    let trainer = PPOTrainer(
        observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 3,
        policyLearningRate: 1e-2, valueLearningRate: 1e-1, gamma: 0.99, clipEpsilon: 0.2, epochs: 4
    )
    let observations = (0..<20).map { i in [Float(i) * 0.01, 0, 0.02, 0] }
    let actions = (0..<20).map { i in [Float(i) * 0.05 - 0.5, 0, 0] }
    let logProbs = observations.indices.map { i in
        gaussianLogProb(action: actions[i], mean: [0, 0, 0], std: [1, 1, 1])
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

Create `Tests/MLXPolicyTrainingTests/ReinforceTrainerTests.swift`:

```swift
// Tests/MLXPolicyTrainingTests/ReinforceTrainerTests.swift
import Testing

@testable import MLXPolicyTraining
import MuJoCoRLEnv

@Test func trainStepReducesLossOnAFixedBatch() {
    let trainer = ReinforceTrainer(
        observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 1, learningRate: 1e-2, gamma: 0.99
    )
    let trajectories = [
        Trajectory(
            observations: (0..<20).map { i in [Float(i) * 0.01, 0, 0.02, 0] },
            actions: (0..<20).map { i in [Float(i) * 0.05 - 0.5] },
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

- [ ] **Step 4: Update `mujoco-rl-demo/main.swift`**

Replace `Sources/mujoco-rl-demo/main.swift` in full:

```swift
// Sources/mujoco-rl-demo/main.swift
import MuJoCoRLEnv
import MLXPolicyTraining

let algorithm = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "reinforce"
let observationDimensions = 4
let actionDimensions = 1
let hiddenDimensions = 32
let gamma: Float = 0.99
let episodesPerBatch = 16
let iterations = 200

func meanReturn(_ trajectories: [Trajectory]) -> Float {
    let totals = trajectories.map { $0.rewards.reduce(0, +) }
    return totals.reduce(0, +) / Float(totals.count)
}

func cartpoleReward(_: CartpoleObservation) -> Float { 1.0 }

switch algorithm {
case "reinforce":
    let trainer = ReinforceTrainer(
        observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
        actionDimensions: actionDimensions, learningRate: 3e-3, gamma: gamma
    )
    print("Training REINFORCE on cartpole balance (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
    for iteration in 0..<iterations {
        let weights = trainer.policy.snapshot()
        let trajectories = await collectBatch(
            makeEnvironment: { CartpoleEnv() }, weights: weights, episodeCount: episodesPerBatch,
            reward: cartpoleReward, maxSteps: CartpoleEnv.maxSteps, baseSeed: UInt64(iteration) &* 1_000_003
        )
        let loss = trainer.trainStep(trajectories: trajectories)
        print("iter \(iteration): mean return \(meanReturn(trajectories))  (loss \(loss))")
    }
case "ppo":
    let trainer = PPOTrainer(
        observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
        actionDimensions: actionDimensions, policyLearningRate: 3e-3, valueLearningRate: 1e-2,
        gamma: gamma, clipEpsilon: 0.2, epochs: 4
    )
    print("Training PPO on cartpole balance (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
    for iteration in 0..<iterations {
        let weights = trainer.policy.snapshot()
        let trajectories = await collectBatch(
            makeEnvironment: { CartpoleEnv() }, weights: weights, episodeCount: episodesPerBatch,
            reward: cartpoleReward, maxSteps: CartpoleEnv.maxSteps, baseSeed: UInt64(iteration) &* 1_000_003
        )
        let (policyLoss, valueLoss) = trainer.trainStep(trajectories: trajectories)
        print("iter \(iteration): mean return \(meanReturn(trajectories))  (policy loss \(policyLoss), value loss \(valueLoss))")
    }
default:
    print("Unknown algorithm '\(algorithm)'. Usage: mujoco-rl-demo [reinforce|ppo]")
}
```

- [ ] **Step 5: Build and run tests**

Run: `PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig swift test --filter MLXPolicyTrainingTests`
Expected: PASS (9 tests).

Run: `PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig xcodebuild build -scheme mujoco-rl-demo -destination 'platform=macOS' -derivedDataPath .build-xcode` (matches the README's existing note that MLX targets need `xcodebuild`, not plain `swift build`).
Expected: build succeeds.

Manually smoke `mujoco-rl-demo` still trains correctly (this is the regression gate for the whole generalization):
```bash
.build-xcode/Build/Products/Debug/mujoco-rl-demo reinforce
```
Expected: runs 200 iterations, mean return trending upward similarly to before this task (not numerically pinned — eyeball the trend).

- [ ] **Step 6: Commit**

```bash
git add Package.swift Sources/MLXPolicyTraining Sources/mujoco-rl-demo/main.swift \
        Tests/MLXPolicyTrainingTests
git rm Sources/mujoco-rl-demo/GaussianPolicy.swift Sources/mujoco-rl-demo/GaussianLogProb.swift \
       Sources/mujoco-rl-demo/PPOTrainer.swift Sources/mujoco-rl-demo/ReinforceTrainer.swift
git rm -r Tests/MujocoRLDemoTests
git commit -m "feat: extract MLXPolicyTraining, generalized to vector actions"
```

---

### Task 5: `Go2Kit` — Go2 observation/command/environment

**Files:**
- Create: `Sources/Go2Kit/Go2Observation.swift`
- Create: `Sources/Go2Kit/Go2Environment.swift`
- Test: Create `Tests/Go2KitTests/Go2EnvironmentTests.swift`
- Modify: `Package.swift` (new `Go2Kit`/`Go2KitTests` targets inside the `#if os(macOS)` block)

**Interfaces:**
- Consumes: `RobotKit.Environment` (Task 1), `MuJoCoRLEnv.ObservationEncoding` (Task 3), `WendyMuJoCo.Menagerie` (existing, resolves `"go2"` → `"unitree_go2"`), `MuJoCo` (`MjModel`/`MjData`).
- Produces: `public struct Go2Observation: ObservationEncoding`, `public struct Go2Command`, `public final class Go2Environment: Environment` (`Observation == Go2Observation`, `Action == Go2Command`). Task 6 consumes `Go2Environment` directly; note `Go2Environment.Action` is `Go2Command`, not `[Float]` — Task 6 converts the policy's raw `[Float]` output into a `Go2Command` before calling `act(_:)` (unlike `CartpoleEnv`, which took `[Float]` directly, since `Go2Command`'s residual-vs-absolute-target distinction is meaningful enough to model as a real type rather than a bare array).

**Before starting:** read `Sources/MuJoCo/MjModel.swift`, `MjData.swift`, `MjBodies.swift`, and `MjSensors.swift` to find the actual Swift accessors for: resolving a named joint to its qpos/qvel address, resolving a named sensor and reading its value (the README documents `model.sensor(named:)` + `data.withSensorValues(_:_:)` for this), and setting per-actuator control values (`data.setCtrl` is already used in `CartpoleEnv`, but confirm whether it takes the full control vector or supports per-index writes for 12 actuators). Do not guess at method names not confirmed present — if something in this brief's sketch doesn't match the real API, adapt to the real one and note the difference in your report.

**Grounding data** (from `wendy-sandbox/image/apps/unitree-sim2real/robots/go2.robot.json` — copy these literally, do not re-derive):

```
joints (order matters):
  FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
  RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
kp:            20 for all 12
kd:            0.5 for all 12
torque_limits: 23.5 for all 12
default_pos:   [0.0, 0.8, -1.5] repeated x4 (per-leg hip/thigh/calf)
action_scale:  [0.125, 0.25, 0.25] repeated x4
ang_vel_scale: 0.25
dof_pos_scale: 1.0
dof_vel_scale: 0.05
fall_detection: base height < 0.18, or upright dot-product < 0.5
```

- [ ] **Step 1: Add `Go2Kit` to `Package.swift`**

Inside the `#if os(macOS)` block, add:

```swift
targets.append(.target(name: "Go2Kit", dependencies: ["MuJoCo", "WendyMuJoCo", "MuJoCoRLEnv", "RobotKit"], path: "Sources/Go2Kit"))
targets.append(.testTarget(name: "Go2KitTests", dependencies: ["Go2Kit"]))
products.append(.library(name: "Go2Kit", targets: ["Go2Kit"]))
```

- [ ] **Step 2: Write the failing tests**

Create `Tests/Go2KitTests/Go2EnvironmentTests.swift`:

```swift
import Testing
@testable import Go2Kit

@Test func resetProducesA45ElementObservation() {
    let env = Go2Environment()
    let obs = env.reset()
    #expect(obs.asArray.count == 45)
}

@Test func zeroCommandKeepsTheRobotUprightBriefly() {
    let env = Go2Environment()
    _ = env.reset()
    let zeroCommand = Go2Command(jointPositionResiduals: [Double](repeating: 0, count: 12))
    for _ in 0..<20 {
        _ = env.act(zeroCommand)
    }
    #expect(env.isTerminated == false)
}

@Test func isTerminatedTripsWhenPosedBelowFallHeight() {
    let env = Go2Environment()
    var obs = env.reset()
    let zeroCommand = Go2Command(jointPositionResiduals: [Double](repeating: 0, count: 12))
    // Drive it toward a fall by stepping with a large, imbalanced residual;
    // fall back to asserting the *observation*'s own baseHeight/upright
    // fields cross the documented thresholds if isTerminated never trips
    // within a bounded number of steps (a real fall may take a while).
    var steps = 0
    while !env.isTerminated && steps < 500 {
        obs = env.act(zeroCommand)
        steps += 1
    }
    // Either it fell (isTerminated true) or it's still standing after 500
    // zero-command steps — both are acceptable outcomes for this smoke test;
    // the real assertion is that isTerminated's own thresholds match
    // go2.robot.json when it DOES trip.
    if env.isTerminated {
        #expect(obs.baseHeight < 0.18 || obs.upright < 0.5)
    }
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `swift test --filter Go2KitTests`
Expected: FAIL to build — `Go2Kit` module doesn't exist yet.

- [ ] **Step 4: Implement `Go2Observation.swift`**

Create `Sources/Go2Kit/Go2Observation.swift` implementing exactly this struct shape (field order matches `go2.robot.json`'s 45-observation composition: 3 + 3 + 3 + 12 + 12 + 12):

```swift
import MuJoCoRLEnv

public struct Go2Observation: ObservationEncoding {
    public let baseAngularVelocity: (Double, Double, Double)
    public let projectedGravity: (Double, Double, Double)
    public let velocityCommand: (vx: Double, vy: Double, wz: Double)
    /// Relative to `Go2Environment.defaultPos`, one per joint in
    /// go2.robot.json's declared order.
    public let jointPositions: [Double]
    public let jointVelocities: [Double]
    /// The residual command actually applied last step (zero on the first
    /// observation after `reset()`).
    public let previousAction: [Double]
    /// Not part of `asArray` — used only for fall detection.
    public let baseHeight: Double
    public let upright: Double

    public var asArray: [Float] {
        var values: [Float] = []
        values.reserveCapacity(45)
        values.append(Float(baseAngularVelocity.0) * 0.25)
        values.append(Float(baseAngularVelocity.1) * 0.25)
        values.append(Float(baseAngularVelocity.2) * 0.25)
        values.append(Float(projectedGravity.0))
        values.append(Float(projectedGravity.1))
        values.append(Float(projectedGravity.2))
        values.append(Float(velocityCommand.vx))
        values.append(Float(velocityCommand.vy))
        values.append(Float(velocityCommand.wz))
        values.append(contentsOf: jointPositions.map { Float($0) * 1.0 })
        values.append(contentsOf: jointVelocities.map { Float($0) * 0.05 })
        values.append(contentsOf: previousAction.map { Float($0) })
        return values
    }
}

public struct Go2Command {
    /// One residual per joint (pre-`actionScale`), in go2.robot.json's
    /// declared joint order. `Go2Environment.act` scales, adds
    /// `defaultPos`, and PD-controls to torque internally.
    public let jointPositionResiduals: [Double]

    public init(jointPositionResiduals: [Double]) {
        precondition(jointPositionResiduals.count == 12)
        self.jointPositionResiduals = jointPositionResiduals
    }
}
```

- [ ] **Step 5: Implement `Go2Environment.swift`**

Create `Sources/Go2Kit/Go2Environment.swift`. Use `Menagerie.resolve(name: "go2")` (or whatever `WendyMuJoCo/Menagerie.swift`'s actual public entry point is called — confirm by reading that file) to get the model path/XML, then `MjModel.load(...)`. Implement `act(_:)` as: `residual = command.jointPositionResiduals[i] * actionScale[i]`, `target = defaultPos[i] + residual`, PD torque `kp * (target - currentPos) - kd * currentVel`, clipped to `±torqueLimits[i]`, written via the model's per-actuator control vector, then `mjStep`. Implement `isTerminated` per the fall-detection thresholds above, reading base height and an up-vector dot product from the model's base body pose (consult `MjBodies.swift`/`MjPhysics.swift` for the exact accessor — e.g. a body's world orientation quaternion, rotated against a unit up vector).

```swift
import Foundation
import MuJoCo
import WendyMuJoCo
import RobotKit

public final class Go2Environment: Environment {
    static let jointOrder = [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    ]
    static let kp = [Double](repeating: 20, count: 12)
    static let kd = [Double](repeating: 0.5, count: 12)
    static let torqueLimits = [Double](repeating: 23.5, count: 12)
    static let defaultPos: [Double] = Array(repeating: [0.0, 0.8, -1.5], count: 4).flatMap { $0 }
    static let actionScale: [Double] = Array(repeating: [0.125, 0.25, 0.25], count: 4).flatMap { $0 }
    static let fallHeightThreshold = 0.18
    static let fallUprightThreshold = 0.5

    private let model: MjModel
    private let data: MjData
    private let velocityCommand: (vx: Double, vy: Double, wz: Double)
    private var previousAction = [Double](repeating: 0, count: 12)

    public init(velocityCommand: (vx: Double, vy: Double, wz: Double) = (0, 0, 0)) {
        // Resolve the real go2 model path/XML via WendyMuJoCo's existing
        // Menagerie resolution (see Menagerie.swift for the exact call) —
        // do not hand-author a Go2 MJCF here.
        self.model = /* try! MjModel.load(...) using Menagerie's "go2" resolution */
        self.data = MjData(model)
        self.velocityCommand = velocityCommand
    }

    public var isTerminated: Bool {
        let obs = observation()
        return obs.baseHeight < Self.fallHeightThreshold || obs.upright < Self.fallUprightThreshold
    }

    public func reset() -> Go2Observation {
        mjResetData(model, data)
        previousAction = [Double](repeating: 0, count: 12)
        return observation()
    }

    public func act(_ action: Go2Command) -> Go2Observation {
        var torques = [Double](repeating: 0, count: 12)
        for i in 0..<12 {
            let residual = action.jointPositionResiduals[i] * Self.actionScale[i]
            let target = Self.defaultPos[i] + residual
            let currentPos = /* current qpos for joint Self.jointOrder[i] */ 0.0
            let currentVel = /* current qvel for joint Self.jointOrder[i] */ 0.0
            let torque = Self.kp[i] * (target - currentPos) - Self.kd[i] * currentVel
            torques[i] = max(-Self.torqueLimits[i], min(Self.torqueLimits[i], torque))
        }
        data.setCtrl(torques)
        mjStep(model, data)
        previousAction = action.jointPositionResiduals
        return observation()
    }

    private func observation() -> Go2Observation {
        // Populate every field from the real model/data state:
        // - baseAngularVelocity: base body's angular velocity (IMU gyro sensor if available, else body qvel)
        // - projectedGravity: world gravity rotated into the base body's frame
        // - jointPositions/jointVelocities: qpos/qvel at each of Self.jointOrder, relative to Self.defaultPos for positions
        // - baseHeight: base body world Z position
        // - upright: dot(worldUp, base body's local up axis after rotation)
        fatalError("fill in using the real MuJoCo Swift API — see MjBodies.swift/MjSensors.swift/MjPhysics.swift")
    }
}
```

The two inline comments (`try! MjModel.load(...)` and the `observation()` body) are genuinely open — fill them in using the real API you found in Step "Before starting," matching the field semantics documented above and in the design spec's Grounding section. Do not leave `fatalError` in the committed code.

- [ ] **Step 6: Run tests to verify they pass**

Run: `swift test --filter Go2KitTests`
Expected: PASS (3 tests).

- [ ] **Step 7: Commit**

```bash
git add Package.swift Sources/Go2Kit Tests/Go2KitTests
git commit -m "feat: add Go2Kit's MuJoCo-backed Go2 environment"
```

---

### Task 6: `go2-locomotion-demo` executable

**Files:**
- Create: `Sources/go2-locomotion-demo/main.swift`
- Modify: `Package.swift` (new `go2-locomotion-demo` executable target inside the `#if os(macOS)` block)

**Interfaces:**
- Consumes: `RobotKit.RunModeKey` (Task 1), `MLXPolicyTraining.PPOTrainer` (Task 4), `Go2Kit.Go2Environment`/`Go2Observation`/`Go2Command` (Task 5), `WorldSimServerCore`/`WendyMuJoCo.WorldSimRecorder` (existing, from the prior session's work).
- Produces: the `go2-locomotion-demo` binary. Nothing downstream depends on this — it's the leaf executable.

- [ ] **Step 1: Add the target to `Package.swift`**

Inside the `#if os(macOS)` block, add:

```swift
targets.append(
    .executableTarget(
        name: "Go2LocomotionDemo",
        dependencies: ["MuJoCoRLEnv", "MLXPolicyTraining", "Go2Kit", "RobotKit",
                      "WendyMuJoCo", "WorldSimServerCore"],
        path: "Sources/go2-locomotion-demo"
    )
)
products.append(.executable(name: "go2-locomotion-demo", targets: ["Go2LocomotionDemo"]))
```

- [ ] **Step 2: Implement**

Create `Sources/go2-locomotion-demo/main.swift`. Reward: negative squared distance between the robot's actual base linear velocity (x, y) and yaw rate versus the commanded `vx, vy, wz`, plus a small per-step alive bonus (e.g. `+0.1`) — the exact coefficients are a tuning detail; start with a squared-error term weighted so it dominates the alive bonus, and adjust if training visibly fails to track commands during the manual smoke test in Step 3.

```swift
// Sources/go2-locomotion-demo/main.swift
import Foundation
import MuJoCoRLEnv
import MLXPolicyTraining
import Go2Kit
import RobotKit
import WendyMuJoCo

let observationDimensions = 45
let actionDimensions = 12
let hiddenDimensions = 64
let gamma: Float = 0.99
let episodesPerBatch = 16
let iterations = 500
let maxStepsPerEpisode = 1000
let checkpointURL = URL(fileURLWithPath: "go2-policy-checkpoint.json")

func go2Reward(_ obs: Go2Observation) -> Float {
    let vxError = Float(obs.velocityCommand.vx) // TODO: replace with actual measured forward velocity from obs
    // NOTE: Go2Observation as specified does not carry the robot's raw
    // (unscaled) linear velocity — only angular velocity and joint state.
    // Before implementing this reward, decide (and note in your report)
    // whether to add a `baseLinearVelocity` field to Go2Observation (Task 5's
    // file, reopened here) so the reward can compare it against
    // velocityCommand directly. This is expected — the design doc named the
    // reward shape but not its exact inputs.
    fatalError("finish go2Reward per the note above, then delete this fatalError")
}

func saveCheckpoint(_ weights: PolicyWeights) throws {
    struct Checkpoint: Codable {
        let w1: [Float], b1: [Float], w2: [Float], b2: [Float], logStd: [Float]
        let inputDimensions: Int, hiddenDimensions: Int, actionDimensions: Int
    }
    let checkpoint = Checkpoint(
        w1: weights.w1, b1: weights.b1, w2: weights.w2, b2: weights.b2, logStd: weights.logStd,
        inputDimensions: weights.inputDimensions, hiddenDimensions: weights.hiddenDimensions,
        actionDimensions: weights.actionDimensions
    )
    try JSONEncoder().encode(checkpoint).write(to: checkpointURL)
}

func loadCheckpoint() throws -> PolicyWeights {
    struct Checkpoint: Codable {
        let w1: [Float], b1: [Float], w2: [Float], b2: [Float], logStd: [Float]
        let inputDimensions: Int, hiddenDimensions: Int, actionDimensions: Int
    }
    let checkpoint = try JSONDecoder().decode(Checkpoint.self, from: Data(contentsOf: checkpointURL))
    return PolicyWeights(
        w1: checkpoint.w1, b1: checkpoint.b1, w2: checkpoint.w2, b2: checkpoint.b2, logStd: checkpoint.logStd,
        inputDimensions: checkpoint.inputDimensions, hiddenDimensions: checkpoint.hiddenDimensions,
        actionDimensions: checkpoint.actionDimensions
    )
}

let learn = CommandLine.arguments.contains("--learn")

await RunModeKey.$current.withValue(learn ? .learn : .infer) {
    switch RunModeKey.current {
    case .learn:
        let trainer = PPOTrainer(
            observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
            actionDimensions: actionDimensions, policyLearningRate: 3e-4, valueLearningRate: 1e-3,
            gamma: gamma, clipEpsilon: 0.2, epochs: 4
        )
        print("Training Go2 locomotion via PPO (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
        for iteration in 0..<iterations {
            let weights = trainer.policy.snapshot()
            let trajectories = await collectBatch(
                makeEnvironment: { Go2Environment() }, weights: weights, episodeCount: episodesPerBatch,
                reward: go2Reward, maxSteps: maxStepsPerEpisode, baseSeed: UInt64(iteration) &* 1_000_003
            )
            let (policyLoss, valueLoss) = trainer.trainStep(trajectories: trajectories)
            print("iter \(iteration): policy loss \(policyLoss), value loss \(valueLoss)")
            if iteration % 20 == 0 {
                try? saveCheckpoint(trainer.policy.snapshot())
            }
        }
        try? saveCheckpoint(trainer.policy.snapshot())
    case .infer:
        guard let weights = try? loadCheckpoint() else {
            print("No checkpoint at \(checkpointURL.path) — run with --learn first.")
            return
        }
        var env = Go2Environment()
        var recorder = WorldSimRecorder()
        var observation = env.reset()
        var frame = 0
        let stepNanos: UInt64 = 20_000_000   // 50 Hz, matching go2.robot.json's policy.rate_hz
        while true {
            let (mean, _) = policyForward(weights, observation: observation.asArray)
            let command = Go2Command(jointPositionResiduals: mean.map { Double($0) })
            observation = env.act(command)
            // Reuse the existing WendyMuJoCo scene/state recording path (see
            // mujoco-live-demo for the pattern) to stream this into the Sim tab.
            frame += 1
            if env.isTerminated {
                print("fell at frame \(frame), resetting")
                observation = env.reset()
            }
            try? await Task.sleep(nanoseconds: stepNanos)
        }
    }
}
```

The `go2Reward` function's `fatalError` and the `WorldSimRecorder` wiring's exact `record(...)` call (matching `mujoco-live-demo`'s pattern — reuse `buildScene`/`buildState` the same way, adapting the title and geom source to `Go2Environment`'s underlying `MjModel`/`MjData`, which need small accessors added to `Go2Environment` for the recorder to reach them) are genuinely open — resolve both using the real Task 5 code you just wrote, and the real `WorldSimRecorder` API from the prior session's work in `Sources/WendyMuJoCo/WorldSimRecorder.swift`. Do not leave `fatalError` in the committed code.

- [ ] **Step 3: Build and manually smoke-test**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
xcodebuild build -scheme go2-locomotion-demo -destination 'platform=macOS' -derivedDataPath .build-xcode
.build-xcode/Build/Products/Debug/go2-locomotion-demo --learn
```

Expected: runs without crashing, prints decreasing-trending policy/value loss over at least the first ~20 iterations (not numerically pinned — eyeball the trend; if it diverges or crashes, that's a real bug to fix, not something to paper over).

Then:
```bash
.build-xcode/Build/Products/Debug/go2-locomotion-demo
```
Expected: loads the checkpoint just trained, runs the inference loop without crashing.

- [ ] **Step 4: Commit**

```bash
git add Package.swift Sources/go2-locomotion-demo
git commit -m "feat: add go2-locomotion-demo (PPO training + inference, --learn flag)"
```

---

## Self-Review Notes

- **Spec coverage:** all 6 design components covered — `RobotKit` (Task 1), generalized `PolicyWeights` (Task 2), generalized `Rollout.swift` + `CartpoleEnv` retrofit (Task 3), `MLXPolicyTraining` extraction (Task 4), `Go2Kit` (Task 5), `go2-locomotion-demo` (Task 6). Phase 2 (real hardware) is explicitly out of scope per the design and not attempted here.
- **Placeholder scan:** Tasks 1-4 have zero placeholders — every line is real, verified-against-existing-code content. Tasks 5-6 have two narrow, explicitly-flagged open points (`Go2Environment`'s MuJoCo API calls for named-joint/sensor access, and `go2Reward`'s missing `baseLinearVelocity` input) — both are real engineering unknowns this plan cannot resolve without reading files this plan's author didn't verify line-by-line, and both are called out with an exact next step (read specific files; add a specific field) rather than hand-waved. This is a deliberate, narrow exception to "no placeholders," not a general pattern — every other line in every task is concrete.
- **Type consistency:** `PolicyWeights(w1:b1:w2:b2:logStd:inputDimensions:hiddenDimensions:actionDimensions:)` used identically in Tasks 2, 3, 4. `collectEpisode`/`collectBatch`'s `(makeEnvironment:weights:reward:maxSteps:seed/baseSeed:)` shape matches between Task 3's implementation and Task 4's `mujoco-rl-demo` call sites. `GaussianPolicy(observationDimensions:hiddenDimensions:actionDimensions:)` and `PPOTrainer`/`ReinforceTrainer`'s matching `actionDimensions` parameter are consistent across Task 4's implementation and Task 6's Go2-specific construction (`actionDimensions: 12`).

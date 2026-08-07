# RobotKit: Go2 locomotion (sim + MLX training) — design

**Date:** 2026-08-07
**Status:** Approved (brainstorming) — pending implementation plan
**Phase:** 1 of 2 (sim-only; Phase 2 is real hardware, named but not designed here)

## Goal

A general-purpose `Environment`/`Policy` abstraction (`RobotKit`), instantiated
first for the Unitree Go2 quadruped, such that:

- `swift run go2-locomotion-demo` runs inference — real control logic, a
  trained (or pre-baked) locomotion policy, MuJoCo physics in the loop.
- `swift run go2-locomotion-demo --learn` trains that policy via PPO against
  the same MuJoCo environment.
- Deterministic app code (workflow orchestration, safety limits, per-subsystem
  command composition) is identical across inference and training, and will
  stay identical when Phase 2 swaps in real hardware — only the `Environment`
  backend and whether a `Policy` is being updated change.

## Non-goals (this phase)

- No real Go2 hardware backend — that's Phase 2 (`Go2HardwareEnvironment`
  over `SwiftROS2`, `WENDY_HAL` env var, `woof.local` as the concrete target).
  Named here so the abstraction is designed with it in mind, not designed now.
- Not porting the existing `go2_policy.onnx` (from
  `wendy-sandbox/image/apps/unitree-sim2real`) into MLX. It stays where it is,
  untouched, as a separately-working reference. This phase's trained policy is
  a new, independent artifact.
- Not touching `unitree-sim2real`'s WSF/bridge/`hil/` pipeline at all — that
  remains the deployment-validation tool it already is.
- No arm/manipulator support — the Go2 in scope has no arm attachment right
  now. `RobotKit`'s per-subsystem composition is designed to support one
  later (a second `Policy` slot alongside legs), but nothing arm-specific is
  built in this phase.

## Architecture

```
                     RunModeKey.@TaskLocal (.infer | .learn)
                                    │
                    ┌───────────────┴───────────────┐
                    ▼                                ▼
         swift run go2-locomotion-demo   swift run go2-locomotion-demo --learn
              (RunModeKey = .infer)              (RunModeKey = .learn)
                    │                                │
                    ▼                                ▼
        ┌─────────────────────┐          ┌─────────────────────────┐
        │ Load trained weights │          │ PPO training loop        │
        │ from disk, run       │          │ (MLXPolicyTraining),      │
        │ inference loop,       │          │ collectBatch against      │
        │ stream to             │          │ Go2Environment, save       │
        │ WorldSimServerCore    │          │ weights to disk on exit   │
        └──────────┬───────────┘          └──────────┬───────────────┘
                    │                                  │
                    └────────────────┬─────────────────┘
                                      ▼
                     ┌───────────────────────────────┐
                     │  Go2Environment: Environment    │
                     │  (Sources/Go2Kit) — wraps        │
                     │  MjModel/MjData for Menagerie    │
                     │  "go2", applies PD torque control│
                     └───────────────┬───────────────┘
                                      │ Go2Observation (45-dim) / Go2Command (12-dim)
                                      ▼
                     ┌───────────────────────────────┐
                     │  Locomotion Policy               │
                     │  (TrainablePolicy, MLX,           │
                     │  Sources/MLXPolicyTraining)        │
                     └───────────────────────────────┘
```

`RobotKit`'s `Environment` protocol is the seam Phase 2 swaps
`Go2Environment` (sim) for `Go2HardwareEnvironment` (real) behind — nothing
above that line changes.

## Grounding: the real Go2 numbers

From `wendy-sandbox/image/apps/unitree-sim2real/robots/go2.robot.json` (the
existing, already-working ONNX policy's own descriptor — reused verbatim so
the new MLX policy trains against the same convention, not an invented one):

- **12 joints/actuators**, order `FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh,
  FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf`.
- **Control mode:** `pd_torque` — the policy outputs a 12-dim residual,
  scaled by `action_scale` (`[0.125, 0.25, 0.25]` per leg, repeated ×4), added
  to `default_pos` (`[0.0, 0.8, -1.5]` per leg, repeated ×4), then PD-controlled
  to torque with `kp=20`/`kd=0.5` per joint, clipped to `torque_limits=23.5`.
- **45-dim observation**, composed exactly as the existing policy's own
  `num_observations: 45` implies (3 + 3 + 3 + 12 + 12 + 12): base angular
  velocity ×`ang_vel_scale` (0.25), projected gravity in body frame, velocity
  command (`vx, vy, wz`) ×`commands_scale`, joint positions relative to
  `default_pos` ×`dof_pos_scale` (1.0), joint velocities ×`dof_vel_scale`
  (0.05), previous action (12).
- **Fall detection** (used as the training episode's termination condition):
  base height `< 0.18` or upright dot-product `< 0.5`.
- **Feet contact sensors** (`touch_FL/FR/RL/RR`) available for observation or
  reward shaping, not required for the minimal first policy.

## Components

### 1. `Sources/RobotKit/` (new library, cross-platform)

Robot-agnostic core — no MLX dependency, no Go2 knowledge.

```swift
public protocol Environment<Observation, Action> {
    associatedtype Observation
    associatedtype Action
    mutating func reset() -> Observation
    mutating func act(_ action: Action) -> Observation
    var isTerminated: Bool { get }
}

public enum RunMode: Sendable, Equatable { case infer, learn }
public enum RunModeKey {
    @TaskLocal public static var current: RunMode = .infer
}
```

`act(_:) -> Observation` returns the resulting observation directly (mirrors
`CartpoleEnv.step`'s shape minus the RL-specific `reward`/`done` — those are
computed by the training harness from `Observation`, not owned by
`Environment`, since reward is a training-time construct that doesn't exist
on real hardware).

### 2. `Sources/MuJoCoRLEnv/PolicyWeights.swift` — generalized in place
   (stays cross-platform, no MLX — unchanged file location)

`PolicyWeights`/`policyForward`/`gaussianLogProb`/`sampleGaussian` are
deliberately MLX-free today specifically so they're `Sendable` and safe to
call from parallel `TaskGroup` rollout workers (per the existing doc
comment). `Rollout.swift`'s generalized `collectEpisode`/`collectBatch`
(component 3 below) take `PolicyWeights` as a parameter, so it — and
everything it needs — must stay cross-platform in `MuJoCoRLEnv`, **not**
move into the macOS-only `MLXPolicyTraining` target below. Generalized from
scalar to vector actions in place:

- `PolicyWeights`: `w2`/`b2`/`logStd` resized from `[1 × hidden]`/scalar to
  `[actionDimensions × hidden]`/`[actionDimensions]`; gains an
  `actionDimensions: Int` field.
- `policyForward(_:observation:) -> (mean: [Float], std: [Float])` — was
  `(Float, Float)`.
- `gaussianLogProb(action: [Float], mean: [Float], std: [Float]) -> Float` —
  sums independent per-dimension log-probs into one joint log-prob (standard
  diagonal-covariance-Gaussian convention for continuous control).
- `sampleGaussian(mean: [Float], std: [Float], ...) -> [Float]`.

### 3. `Sources/MLXPolicyTraining/` (new library, macOS-only, alongside the
   existing `#if os(macOS)` mlx-swift gate)

**Moved** from `Sources/mujoco-rl-demo/` — `GaussianPolicy.swift`,
`PPOTrainer.swift`, `ReinforceTrainer.swift`, `GaussianLogProb.swift` — and
**generalized from scalar to vector actions** in the same move (real work,
not a mechanical copy). These four (unlike `PolicyWeights` above) are
inherently MLX-dependent already, so moving them costs nothing
cross-platform-wise:

- `GaussianPolicy`: `fc2 = Linear(hiddenDimensions, actionDimensions)`
  (was hardcoded `1`); `logStd: MLXArray` becomes shape `[actionDimensions]`
  (was a 0-d scalar); gains an `actionDimensions: Int` stored property;
  `snapshot() -> PolicyWeights` carries the new field through.
- `gaussianLogProbMLX`: sums over the action axis (`.sum(axis: -1,
  keepDims: true)`) instead of relying on an implicit width-1 axis.
- `PPOTrainer`/`ReinforceTrainer`: `actionsArray` shaped
  `[count, actionDimensions]` instead of `[count, 1]`; otherwise unchanged —
  `gaussianLogProbMLX` already returns the summed joint log-prob per row.
- `Trajectory.actions: [[Float]]` (was `[Float]`) — one action vector per
  step. (`Trajectory` itself stays in `MuJoCoRLEnv/Rollout.swift` — it's
  already MLX-free and used by both the cross-platform rollout collector and
  the MLX trainers.)

`Sources/mujoco-rl-demo/` keeps only its `main.swift` (CartpoleEnv-specific
CLI), now depending on `MLXPolicyTraining` and constructing
`GaussianPolicy`/`PPOTrainer`/`ReinforceTrainer` with `actionDimensions: 1` —
**no behavior change** to the existing cartpole demo.

### 4. `Sources/MuJoCoRLEnv/Rollout.swift` — generalized off `CartpoleEnv`

Currently hardcodes `CartpoleEnv()` construction and its exact
`reset()`/`step(action: Float)` shape. Generalized to run against any
`RobotKit.Environment` whose `Action == [Float]` and whose `Observation`
supplies a flat encoding for the policy:

```swift
public protocol ObservationEncoding {
    var asArray: [Float] { get }
}

public func collectEpisode<E: Environment>(
    makeEnvironment: () -> E, weights: PolicyWeights,
    reward: (E.Observation) -> Float, maxSteps: Int, seed: UInt64
) -> Trajectory where E.Action == [Float], E.Observation: ObservationEncoding

public func collectBatch<E: Environment>(
    makeEnvironment: @escaping @Sendable () -> E, weights: PolicyWeights, episodeCount: Int,
    reward: @escaping @Sendable (E.Observation) -> Float, maxSteps: Int, baseSeed: UInt64
) async -> [Trajectory] where E.Action == [Float], E.Observation: ObservationEncoding
```

Termination is `env.isTerminated || steps >= maxSteps` — checked directly
against the environment `collectEpisode` already owns, not threaded through
as a separate closure. An earlier draft of this design proposed a caller-
supplied `isDone: (Observation) -> Bool` alongside `env.isTerminated`, but
that can't actually express "stop when the environment says so" (it only
sees the observation, not the environment instance) without duplicating
`isTerminated`'s logic externally — so it's dropped. Only `reward` is
supplied by the caller (genuinely training-task-specific — the same
environment could be trained against different reward shaping). `CartpoleEnv`
is retrofitted to conform to
`Environment` with `Action == [Float]` (a single-element array, not bare
`Float` — its `act(_:)` unwraps `action[0]` before calling the existing
internal physics step) and `Observation == CartpoleObservation` (already
gains `asArray` today, so it already satisfies `ObservationEncoding` as-is).
Its existing `reset()`/`step()` physics logic is unchanged internally, just
exposed through the protocol's shape — the old `step`'s reward (always `1.0`)
moves to the training call site's `reward` closure, and its `done` (out of
bounds OR step-count cap) becomes `isTerminated`, computed the same way but
now queried by `collectEpisode` directly against the environment instead of
returned inline. This retrofit is both required (so `mujoco-rl-demo` keeps
working through the generalized `Rollout.swift`) and serves as the
validation that `Environment` actually fits a second, independent robot.

### 5. `Sources/Go2Kit/` (new library, macOS-only — depends on `MuJoCoRLEnv`'s
   `ObservationEncoding` and `RobotKit`'s `Environment`)

```swift
public struct Go2Observation: ObservationEncoding {
    public let baseAngularVelocity: (Double, Double, Double)
    public let projectedGravity: (Double, Double, Double)
    public let velocityCommand: (vx: Double, vy: Double, wz: Double)
    public let jointPositions: [Double]   // 12, relative to defaultPos
    public let jointVelocities: [Double]  // 12
    public let previousAction: [Double]   // 12
    public let baseHeight: Double         // for fall detection, not in asArray
    public let upright: Double            // dot(worldUp, bodyUp), for fall detection
    public var asArray: [Float] { /* the 45-value concatenation above, pre-scaled */ }
}

public struct Go2Command {
    public let jointPositionResiduals: [Double]   // 12, pre-action_scale
}

public final class Go2Environment: Environment {
    public init(velocityCommand: (vx: Double, vy: Double, wz: Double) = (0, 0, 0))
    public func reset() -> Go2Observation
    public func act(_ action: Go2Command) -> Go2Observation
    public var isTerminated: Bool   // fall detection: height < 0.18 || upright < 0.5
}
```

`act(_:)` internally: `residual = command.jointPositionResiduals *
actionScale`, `target = defaultPos + residual`, PD torque via `kp`/`kd`
against current joint pos/vel, clipped to `torqueLimits`, applied via
`data.setCtrl`, then `mjStep`. All numeric constants (`actionScale`,
`defaultPos`, `kp`, `kd`, `torqueLimits`, fall-detection thresholds) are the
literal values from `go2.robot.json` above, not re-derived.

Model loading reuses `WendyMuJoCo`'s existing `Menagerie.swift` (`"go2"` →
`"unitree_go2"`) — no new model-fetch code.

### 6. `Sources/go2-locomotion-demo/` (new executable, macOS-only)

```swift
let learn = CommandLine.arguments.contains("--learn")
await RunModeKey.$current.withValue(learn ? .learn : .infer) {
    switch RunModeKey.current {
    case .learn:
        // PPOTrainer(observationDimensions: 45, hiddenDimensions: H, actionDimensions: 12, ...)
        // loop: collectBatch(makeEnvironment: { Go2Environment() }, weights: trainer.policy.snapshot(),
        //                     reward: <forward-velocity-tracking reward>, maxSteps: ..., ...)
        //       trainer.trainStep(trajectories:)
        // save trainer.policy.snapshot() to disk (JSON or a simple binary format) on exit / periodically
    case .infer:
        // load PolicyWeights from disk, run Go2Environment in a loop at the policy's rate_hz (50 Hz),
        // stream through WorldSimServerCore's WorldSimRecorder (same pattern as mujoco-live-demo)
        // so the result is watchable in the Sim tab
    }
}
```

Reward for the first trained policy: forward-velocity tracking (penalize the
distance between the robot's actual base velocity and the commanded
`vx, vy, wz`, matching what the existing ONNX policy was itself trained to
do) plus a small alive bonus. Exact reward-shaping coefficients are an
implementation-time tuning detail, not fixed here.

## Data flow (inference)

1. `main` sets `RunModeKey.current = .infer`, loads `PolicyWeights` from a
   checkpoint file.
2. Loop: `Go2Environment.observe()`-equivalent (via the last `act` return, or
   an explicit `reset()` on start) → `policyForward(weights:observation:)` →
   `Go2Command` → `Go2Environment.act(_:)` → `WorldSimRecorder.record(...)`.
3. Paced to the policy's `rate_hz` (50 Hz) — matching `go2.robot.json`, not
   MuJoCo's own `physics.timestep`/`decimation` (4 physics steps per policy
   step, per the descriptor).

## Data flow (training)

1. `main` sets `RunModeKey.current = .learn`, constructs a fresh `PPOTrainer`.
2. Loop: `collectBatch` runs N parallel episodes against fresh
   `Go2Environment` instances (each owns its own `MjModel`/`MjData` — neither
   is `Sendable`, matching the existing cartpole pattern's own doc comment)
   using the trainer's current weight snapshot; `trainStep` updates the MLX
   model from the batch; repeat for a fixed iteration count or until reward
   plateaus (implementation-time tuning, not fixed here).
3. Weights saved to disk at the end (and optionally periodically), in
   whatever format `PolicyWeights` round-trips through (plain `Codable` JSON
   is sufficient at this size — 45×H + H×12 + biases, no need for a binary
   format).

## Error handling & edge cases

- **Episode termination (fall):** `Go2Environment.isTerminated` — training's
  `collectEpisode` treats this as `done` like `maxSteps`; the inference loop
  checks it every step and calls `reset()` when true (logging the fall), so
  a long-running `swift run go2-locomotion-demo` session recovers on its own
  instead of driving a fallen robot forever.
- **`MjModel`/`MjData` not `Sendable`:** unchanged constraint from the
  existing cartpole code — `collectBatch`'s generalized signature takes
  `makeEnvironment: () -> E` specifically so each parallel worker constructs
  its own, never sharing one across the `TaskGroup` boundary.
- **Checkpoint format drift:** `PolicyWeights` gaining `actionDimensions`
  breaks decoding any old scalar-only checkpoint — acceptable, since no
  scalar-action checkpoint for a 12-dim robot could have existed anyway,
  and `mujoco-rl-demo`'s own cartpole runs don't currently persist
  checkpoints to disk (trains fresh every run).
- **Reward closure concurrency:** `collectBatch`'s `makeEnvironment`/`reward`
  closures run inside a `TaskGroup` worker per episode, so both are typed
  `@Sendable` — pure functions closing over no mutable state, which the
  forward-velocity reward is.

## Testing

- **`RobotKitTests`:** `RunModeKey` defaults to `.infer`; `withValue` scoping
  behaves as `TaskLocal` guarantees (a unit test mostly documenting the
  contract, since `@TaskLocal` itself is stdlib-verified).
- **`MLXPolicyTrainingTests`** (macOS-only): `policyForward`/`sampleGaussian`/
  `gaussianLogProb` produce correctly-shaped vector output for
  `actionDimensions > 1`; `gaussianLogProbMLX`'s summed log-prob matches the
  scalar formula's sum across independently-computed per-dimension calls
  (cross-check the MLX path against the plain-Swift path, the same way the
  existing tests presumably already do for the scalar case — verify at
  implementation time and preserve that comparison).
- **`mujoco-rl-demo`'s existing tests** (`GaussianPolicyTests`,
  `PPOTrainerTests`, `ReinforceTrainerTests`) continue passing unmodified
  after the move+generalization, constructing with `actionDimensions: 1` —
  this is the regression gate proving the generalization didn't change
  scalar-case behavior.
- **`Go2KitTests`:** `Go2Environment.reset()` yields a 45-element `asArray`;
  `act(_:)` with a zero command keeps the robot upright for at least N steps
  (sanity check the PD-torque translation is wired correctly against the
  real `kp`/`kd`/`torque_limits`); `isTerminated` trips when `MjData` is
  manually posed below the fall-detection height threshold.
- **Manual smoke:** `swift run go2-locomotion-demo --learn` for a short
  iteration count produces a non-crashing training run with decreasing
  policy loss trend (not asserted numerically — eyeballed); `swift run
  go2-locomotion-demo` after that loads the checkpoint and the Go2 is
  visible standing/moving in the Sim tab via the existing
  `wendy-worldsim-server`/desktop-native bridge from the prior session's work.

## Phase 2 (named, not designed)

`Go2HardwareEnvironment: Environment` over `SwiftROS2`/`SwiftROS2Msgs`
(already built in `wendy-sandbox/image/apps/unitree-sim2real/consumer`),
talking directly to the real Go2's onboard DDS network (reachable at
`woof.local`) — no WSF/bridge/`hil/` involved, since the real robot is
already a native ROS2/DDS peer. Switch controlled by an explicit `WENDY_HAL`
env var (default `sim`), mirroring the `WENDY_LOCAL_SIM_PORT` convention
already established for wendy-sandbox's desktop-native bypass. Safety-limit
validation (rate limits, joint bounds, e-stop) before anything sim-trained
touches the real robot is a hard requirement of that phase, not optional
polish.

## Affected files (summary)

- New: `Sources/RobotKit/Environment.swift` (+ `Tests/RobotKitTests/`)
- Modify: `Sources/MuJoCoRLEnv/PolicyWeights.swift` (generalized to vector actions, in place — stays cross-platform, does NOT move)
- New: `Sources/MLXPolicyTraining/{GaussianPolicy,PPOTrainer,ReinforceTrainer,GaussianLogProb}.swift`
  (moved + generalized from `mujoco-rl-demo`; `PolicyWeights`/`Trajectory` are NOT part of this move — see above)
- Moved (not duplicated): `Tests/MujocoRLDemoTests/{GaussianPolicyTests,PPOTrainerTests,ReinforceTrainerTests}.swift`
  → `Tests/MLXPolicyTrainingTests/`, updated to `@testable import MLXPolicyTraining` and
  `actionDimensions: 1`. `Tests/MujocoRLDemoTests/` and its `Package.swift` test target are
  removed — `main.swift` has no testable logic of its own once the trainers move out.
- Modify: `Sources/MuJoCoRLEnv/Rollout.swift` (generalized `collectEpisode`/`collectBatch`)
- Modify: `Sources/MuJoCoRLEnv/CartpoleEnv.swift` (conform to `RobotKit.Environment`)
- Modify: `Sources/mujoco-rl-demo/main.swift` (depend on `MLXPolicyTraining`, `actionDimensions: 1`)
- New: `Sources/Go2Kit/{Go2Observation,Go2Command,Go2Environment}.swift` (+ `Tests/Go2KitTests/`)
- New: `Sources/go2-locomotion-demo/main.swift`
- Modify: `Package.swift` (new targets/products: `RobotKit`, `MLXPolicyTraining`, `Go2Kit`, `go2-locomotion-demo`; `mujoco-rl-demo` gains a `MLXPolicyTraining` dependency)

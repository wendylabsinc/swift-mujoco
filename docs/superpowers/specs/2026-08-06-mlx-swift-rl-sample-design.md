# MLX-Swift RL sample: design

## Context

`swift-mujoco` provides low-level MuJoCo bindings (stepping, control, sensors,
state save/restore, offscreen rendering) but nothing resembling an RL
framework — no environment abstraction, no learning algorithm, no policy
network. This spec covers a self-contained sample project that demonstrates
training a policy with [MLX-Swift](https://github.com/ml-explore/mlx-swift)
against a MuJoCo-simulated task, as a template for building real RL work on
top of this library.

## Goal

A new executable, `mujoco-rl-demo`, that trains a continuous-control policy
to balance a cartpole using two algorithms — REINFORCE and PPO — and prints
the mean episode return per training iteration so the reward curve's upward
trend is visible in the terminal.

## Task: cartpole balance

A small custom MJCF, not an external model file:

- Cart: slide joint along the x-axis, actuated by a motor (continuous force).
- Pole: hinge joint attached to the cart, unactuated, free to swing.
- Observation (4-dim): cart position, cart velocity, pole angle, pole angular
  velocity — read from `qpos`/`qvel`.
- Action: continuous force on the cart's motor (this is why the policy is a
  Gaussian over a continuous action, not a categorical left/right policy
  like classic gym `CartPole` — MuJoCo's actuator is continuous, and fighting
  that with a discretized action space would be an unnatural fit).
- Reward: +1 per step while the pole angle and cart position stay within
  bounds.
- Episode end: pole falls past an angle threshold, cart goes off-track, or a
  max step count (500) is reached.

## Concurrency constraint

`MjModel` and `MjData` are deliberately **not** `Sendable` in this codebase
(see the doc comments on both classes) — they wrap mutable MuJoCo C state
that isn't safe to touch from multiple isolation domains. Parallel rollout
collection therefore cannot share one `MjModel` across worker threads/tasks;
each worker loads its own `MjModel`+`MjData` pair from the same XML string.
For cartpole this is cheap (sub-millisecond to compile per worker), so it's
not a meaningful cost.

## Architecture

**Inference is separated from training.** MLX's thread-safety for
concurrent forward passes across OS threads is unverified for this project,
and parallel rollout collection shouldn't be built on an unverified
assumption. So the forward pass used *during rollout* — sampling an action
at each simulation step — is a small hand-rolled Swift function operating on
a plain snapshot of the policy weights (`[Float]` arrays), with no MLX
involved. MLX (`MLXNN`/`MLXOptimizers`) is used only for the actual learning
step: recomputing log-probs with gradient tracking over a batch of stored
`(obs, action)` pairs, and taking an Adam step. This isn't just a
concurrency workaround — it's how REINFORCE/PPO naturally factor: gradients
are never needed while acting, only when updating. It also makes rollout
workers trivially `Sendable`-clean, since no MLX object ever crosses a
thread boundary.

**Env and policy representation are shared; the two algorithms are separate
trainers.** `CartpoleEnv` and the policy weight snapshot type are shared
infrastructure; `ReinforceTrainer` and `PPOTrainer` are independent types
built on top of them, so either training loop can be read end-to-end without
cross-referencing the other.

## Components

- **`CartpoleEnv`** — owns one `MjModel`/`MjData` pair.
  `reset() -> [Float]` (4-dim observation), `step(action: Float) -> (obs:
  [Float], reward: Float, done: Bool)`.
- **`GaussianPolicy`** — an `MLXNN.Module`: `Linear(4→32) → tanh →
  Linear(32→1)` producing the action mean, plus a learned scalar log-std.
- **`PolicyWeights`** — a `Sendable` plain-value snapshot (`[Float]` arrays)
  extracted from `GaussianPolicy` after each training update; this is what
  the hand-rolled Swift forward function in rollout workers consumes.
- **`RolloutWorker`** — owns one `CartpoleEnv`; given a `PolicyWeights`
  snapshot, runs a number of episodes sequentially and returns trajectories
  (plain arrays of obs/action/reward, plus the closed-form Gaussian log-prob
  at sampling time — needed by PPO's probability ratio).
- **`ParallelRolloutCollector`** — fans out to N `RolloutWorker`s via a Swift
  `TaskGroup` (N ≈ core count), gathering trajectories into one batch per
  training iteration.
- **`ReinforceTrainer`** — discounted returns with a batch-mean baseline;
  `valueAndGrad` over `-mean(logProb * advantage)`; Adam step.
- **`PPOTrainer`** — adds a small value-network head for the baseline, and a
  clipped surrogate objective comparing stored old log-probs against
  log-probs recomputed (with gradient) on the current weights, over a few
  epochs per batch. No GAE — kept intentionally minimal to stay sample-sized.
- **`main.swift`** — `swift run mujoco-rl-demo [reinforce|ppo]` (default
  `reinforce`); prints mean episode return per iteration.

## Package and CI impact

A new executable target `mujoco-rl-demo` is added to the existing
`Package.swift`, depending on `MuJoCo` plus `MLX`/`MLXNN`/`MLXOptimizers`
from `ml-explore/mlx-swift`. Since MLX is Apple-Metal-only, the new target,
its product, and the `mlx-swift` package dependency declaration itself are
all wrapped in `#if os(macOS)` inside `Package.swift` — on Linux, the
manifest never mentions `mlx-swift`, so Linux CI never resolves or attempts
to build it. No CI YAML changes are needed: the existing `swift build`/`swift
test` steps on Linux simply won't see the target.

## Testing

- `CartpoleEnvTests` (cross-platform, no MLX dependency): reset/step
  mechanics, reward and termination conditions. Fast and deterministic —
  runs as part of `swift test` on both Linux and macOS.
- No automated test asserts that either algorithm actually learns to balance
  the pole — that's stochastic and slow to verify in CI. The demo is
  verified by running it and observing the printed mean-return curve trend
  upward, in the same spirit as the existing `mujoco-demo` executable.

## Out of scope

- Vectorized/batched MuJoCo stepping beyond per-worker independent
  `MjModel`/`MjData` pairs.
- GAE, entropy bonuses, learning-rate schedules, or other PPO refinements
  beyond the minimal clipped-surrogate + value-baseline version described
  above.
- Any task other than cartpole (e.g. Menagerie models) — out of scope for
  this sample; a natural follow-up once this pattern exists.
- A Python-side bridge or IPC to any other ML framework.

# swift-mujoco

Swift bindings for the MuJoCo physics engine.

## Prerequisites
Install MuJoCo (headers + shared lib) from the pip wheel. The system Python may not have
a `mujoco` wheel available (e.g. Python 3.14) — use a Python 3.12 environment via `uv`:

    uv venv --python 3.12 .venv-mujoco
    uv pip install --python .venv-mujoco/bin/python mujoco
    PYTHON=.venv-mujoco/bin/python MUJOCO_PREFIX="$HOME/.local" ./Scripts/install-mujoco.sh

This installs headers/lib into `$HOME/.local` (no `sudo` required) and writes
`$HOME/.local/lib/pkgconfig/mujoco.pc`.

Requires macOS 14+ on Apple platforms: `mlx-swift` (used only by the
`MujocoRLDemo` target) needs macOS 14, and SwiftPM requires the package's
declared platform floor to satisfy every dependency's minimum, so
`Package.swift` declares `.macOS(.v14)` package-wide even though the
`MuJoCo`/`WendyMuJoCo` libraries themselves don't need it.

## Build & test
    export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
    swift build
    swift test --skip MLXPolicyTrainingTests

`MLXPolicyTrainingTests` needs Xcode's build system — see
[RL sample (MLX-Swift)](#rl-sample-mlx-swift) below.

## What's here

- `MuJoCo` — Swift bindings: model/data, physics, sensors, raycasting, offscreen
  rendering, spec composition.
- `WendyMuJoCo` — JSON scene/state streaming for the Wendy Sim tab, plus
  Menagerie model resolution.
- `mujoco-rl-demo` — REINFORCE and PPO trained against a cartpole balance
  task using [MLX-Swift](https://github.com/ml-explore/mlx-swift). macOS
  only (MLX is Metal-backed); the target and its `mlx-swift` dependency are
  gated behind `#if os(macOS)` in `Package.swift` so Linux CI never sees
  them.
- `RobotKit` — sim-to-real framework: canonical robot state/command types, the
  observation encoder and action decoder that define the sim-to-real contract,
  transports, and the control loop; also hosts the shared RL plumbing
  (`Environment` protocol, rollout collection, run-mode signaling) used by the
  cartpole and Go2 RL demos. `RobotKitGo2` adds Unitree Go2 messages, CRC, and
  the adapter; `RobotKitSim` drives a MuJoCo Go2 that speaks the robot's own
  wire messages; `RobotKitROS2` provides the DDS transport.
- `Go2Kit` — `Go2Environment`, a MuJoCo-backed Unitree Go2 quadruped driven
  by a joint-space PD controller, loading the real `unitree_go2` model via
  `WendyMuJoCo.Menagerie`. macOS only, same reason as `MLXPolicyTraining`
  below.
- `MLXPolicyTraining` — MLX-Swift policy/value networks and PPO training
  step shared by `mujoco-rl-demo` and `go2-locomotion-demo`. macOS only
  (MLX is Metal-backed), gated behind `#if os(macOS)` alongside the other
  MLX-dependent targets.
- `go2-locomotion-demo` — trains a Go2 locomotion policy via PPO and runs it
  live. See [Go2 locomotion demo (MLX-Swift)](#go2-locomotion-demo-mlx-swift)
  below. macOS only, same reason as `mujoco-rl-demo`.

### Sensors

    let imu = model.sensor(named: "imu-gyro")!
    data.withSensorValues(imu) { v in /* v: UnsafeBufferPointer<Double>, no allocation */ }
    switch imu.kind { case .gyro: ...; default: ... }

### Raycasting / lidar

    let pattern = LidarPattern(azimuthCount: 180, elevationCount: 16,
                               azimuthSpanRadians: 2 * .pi,
                               elevationSpanRadians: .pi / 2)
    let batch = MjRayBatch(capacity: pattern.rayCount)
    let hits = batch.cast(model: model, data: data,
                          origin: data.sitePos(lidarSite),
                          directions: pattern.directions)

### Offscreen cameras

Requires a GL implementation at runtime. On Linux install `libegl1` and, for
CPU-only rendering, `libgl1-mesa-dri`, then set `EGL_PLATFORM=surfaceless` and
`LIBGL_ALWAYS_SOFTWARE=1`. Check `MjOffscreenRenderer.isAvailable` to degrade
gracefully where there is none.

    let r = try MjOffscreenRenderer(model: model, width: 640, height: 480)
    let frame = try r.render(data: data, cameraId: camId)
    // frame.rgb  : [UInt8], w*h*3, rgb8, top-down
    // frame.depth: [Float], w*h, metres, top-down, .infinity where nothing was hit

## CI

`.github/workflows/ci.yml` builds and tests on Linux (amd64 + arm64, via Mesa
llvmpipe software rendering) and macOS on every push to `main` and every pull
request. The Linux job sets `SWIFT_MUJOCO_REQUIRE_GL=1` so a runner that loses
its GL stack fails the build instead of silently skipping the render tests.

## Allocation behaviour

The hot paths are allocation-free; the ergonomic paths are not, and each says which
it is. The rule of thumb: anything returning `[Double]` allocates, anything
returning a value type or taking caller storage does not.

| Instead of | Use | Why |
|---|---|---|
| `data.qpos` | `data.qposSpan`, or `withQpos { }` | the Array is rebuilt per access |
| `data.xmat(i)` / `bodyMat(i)` | `data.bodyMatrix(i)` → `Mat3` | `Mat3` stores nine scalars inline |
| `geomXmat`/`siteMat`/`camMat` | `geomMatrix`/`siteMatrix`/`camMatrix` | same |
| `data.getFullState()` | `data.readFullState(into:)` | reuse one buffer across an MPC loop |
| `renderer.render(data:cameraId:)` | `render(data:cameraId:into:)` + `FrameBuffer` | ~1.7 MB/frame at 640×480 otherwise |

`Vec3`/`Quat`/`Mat3` arithmetic is `@inlinable`, so it inlines into consumer code
rather than crossing a module boundary per operation.

`Mat3` stores its nine doubles in an `InlineArray` (`[9 of Double]`) — fixed-size,
no indirection, no refcount. `InlineArray` is `@available(macOS 26, *)`, which is
why the package declares `platforms: [.macOS(.v26)]`; **Linux is unaffected**, since
`platforms:` only constrains Apple platforms, and Linux is where this actually
deploys. CI's macOS job therefore needs a `macos-26` runner or newer.

Two `InlineArray` quirks that surface in the API: it is not a `Collection` (index
`m` directly, or use `Mat3.span`/`Mat3.array`) and it has no `Equatable`
conformance, so `Mat3.==` is written out rather than synthesised.

### `Span` accessors are experimental

`qposSpan`, `qvelSpan`, `qaccSpan`, `ctrlSpan`, `sensordataSpan` and
`sensorValuesSpan(_:)` return a borrowed `Span`, which the compiler prevents from
outliving the `MjData` — stronger than the `with*` closures, which hand out an
`UnsafeBufferPointer` you are merely asked not to escape.

**Calling them needs nothing special.** *Building this package* does: producing a
`Span` from a class wrapping a raw C pointer currently needs the `Lifetimes`
experimental feature, the underscored `@_lifetime`, and the stdlib-internal
`_overrideLifetime`. All three live in `Sources/MuJoCo/MjSpan.swift` and nowhere
else, so if a toolchain changes their spelling that one file can be deleted without
touching the rest of the library. That is also why the `with*` closure accessors
remain — the primary hot-path API does not depend on unstable spellings.

Note that `Span` is not a `Collection` in Swift 6.3: no `map`/`reduce`/`first`, no
`Array(span)`. Iterate `for i in span.indices`.

## Concurrency

Structured only — no detached tasks, no `DispatchQueue`, no completion handlers.
`collectBatch` fans rollouts out over a `TaskGroup`.

`MjModel`, `MjData`, `MjSpec`, `MjRayBatch`, `MjOffscreenRenderer` and `Handle` are
deliberately **not** `Sendable`: they wrap mutable C state that is unsafe to touch
from two isolation domains. Keep each on one. Everything that is *data* —
`Vec3`, `Mat3`, `Quat`, `RayHit`, `Contact`, `RenderedFrame`, `FrameBuffer`,
`PolicyWeights`, `LidarPattern`, the `*Info` structs — is a `Sendable` value type
and crosses freely.

`Handle` has two forms of the frame pump. Use `syncAsync()` from a `Task`: while the
sim is paused it suspends, whereas `sync()` parks the calling thread with
`Thread.sleep` and would hold a cooperative-pool thread (one per core) for the whole
pause. `syncAsync()` is also cancellation-aware.

## RL sample (MLX-Swift)

The RL demo target is **opt-in**, behind `MUJOCO_RL_DEMO=1`. It is not gated on
`#if os(macOS)`, because a manifest's `#if os(...)` describes the machine running
SwiftPM rather than the build target — so on a Mac cross-compiling for Linux
ARM64 (what `wendy run` does when building a Swift app for a device) that branch
was true and dragged Metal-only `mlx-swift` into a Linux build graph. Without the
variable, `MujocoRLDemo`, `MujocoRLDemoTests` and the `mlx-swift` dependency are
absent from the package entirely.

Because the dependency is conditional, `Package.resolved` only carries pins for
the default (RL-demo-off) configuration. Building with `MUJOCO_RL_DEMO=1` adds
`mlx-swift` pins locally — **don't commit that diff**; it gets pruned again by the
next default build. `mlx-swift` is pinned `exact:` in `Package.swift` so the
opt-in build stays reproducible without them.

`mlx-swift` also can't execute at runtime under plain `swift build`/`swift run`
— SwiftPM's command-line build can't compile the Metal shaders it needs
(see [mlx-swift's README](https://github.com/ml-explore/mlx-swift#readme)).
Build and run it via `xcodebuild` instead:

    export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
    export MUJOCO_RL_DEMO=1
    xcodebuild build -scheme mujoco-rl-demo -destination 'platform=macOS' -derivedDataPath .build-xcode
    .build-xcode/Build/Products/Debug/mujoco-rl-demo reinforce   # or: ppo

Trains a Gaussian policy to balance a cartpole. Rollout collection runs
`CartpoleEnv` episodes in parallel across a `TaskGroup`. Each worker owns its own
`MjData`; the compiled `MjModel` is shared process-wide, because compiling it per
episode (which writes and re-parses a temp MJCF file) cost more than the physics
it was setting up. Sharing is sound because `mj_step` takes `mjModel` as `const`
— but `MjModel`'s lazy introspection caches are mutable, so they are pre-warmed
on a single thread before the model is published and only ever read afterwards.
MLX is only used for the actual gradient step — action sampling during rollout is
a hand-rolled Swift forward pass over a plain snapshot of the policy weights, in
`Sources/MuJoCoRLEnv/`.

`MLXPolicyTrainingTests` (the tests covering the MLX-dependent pieces) can't run
under plain `swift test` for the same reason — verify them locally with

    xcodebuild test -scheme swift-mujoco-Package -destination 'platform=macOS' \
      -skipPackagePluginValidation

(`-skipPackagePluginValidation` is required for the whole-package scheme
specifically: it builds `WorldSimServerCore`, which uses the
`JSONSchemaPlugin` build-tool plugin, and a headless `xcodebuild` invocation
with no prior DerivedData has no way to grant that plugin's one-time trust
approval otherwise — confirmed by running the command above both with and
without the flag. Single-target schemes that don't touch
`WorldSimServerCore`, like `mujoco-rl-demo` and `go2-locomotion-demo`
themselves, build fine without it.)

CI runs `swift test --skip MLXPolicyTrainingTests` and never executes this
target's tests.

## Go2 locomotion demo (MLX-Swift)

`go2-locomotion-demo` wires `RobotKit` + `MuJoCoRLEnv` + `MLXPolicyTraining` +
`Go2Kit` into a runnable PPO training/inference loop for a MuJoCo-simulated
Unitree Go2 quadruped. Like `mujoco-rl-demo` above, it links
`MLXPolicyTraining` (MLX-Swift, Metal-backed), so it needs `xcodebuild` for
the same reason — a plain `swift build`/`swift run` compiles it fine but
fails at runtime (`MLX error: Failed to load the default metallib`) because
SwiftPM's command-line build can't compile the Metal shaders MLX needs.

    export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
    xcodebuild build -scheme go2-locomotion-demo -destination 'platform=macOS' -derivedDataPath .build-xcode
    .build-xcode/Build/Products/Debug/go2-locomotion-demo --learn   # train
    .build-xcode/Build/Products/Debug/go2-locomotion-demo           # run inference from the checkpoint

Unlike the whole-package test command above, building just the
`go2-locomotion-demo` scheme does **not** need `-skipPackagePluginValidation`
— this target doesn't depend on `WorldSimServerCore`/`JSONSchemaPlugin` at
all (see the WorldSim paragraph below), so there's no plugin to validate.

`--learn` trains a Go2 locomotion policy via PPO (`PPOTrainer`, in
`MLXPolicyTraining`) against `Go2Environment`, checkpointing weights to
`go2-policy-checkpoint.json` every 20 iterations and on exit. Without
`--learn`, it loads that checkpoint and runs the policy live. **The
checkpoint path is relative to the current working directory**, not to the
binary — `--learn` and the inference run both need to be invoked from the
same directory, or inference will report `No checkpoint at <path> — run
with --learn first.` and refuse to start.

Both modes stream the running sim into `WendyMuJoCo.WorldSimRecorder`'s slot
files for the Sim tab, exactly like `mujoco-live-demo`. Unlike
`mujoco-live-demo`, though, `go2-locomotion-demo` does **not** host the
WorldSim HTTP router itself (it has no `WorldSimServerCore` dependency at
all) — to actually watch it in the Sim tab you need a separately running
`wendy-worldsim-server` (`swift run wendy-worldsim-server`, or the
`xcodebuild`-built equivalent) serving those slot files.

## Sim-to-real demo (Go2)

    # one-time: fetch the Go2 model
    git clone --depth 1 --filter=blob:none --sparse \
      https://github.com/google-deepmind/mujoco_menagerie .cache/mujoco_menagerie
    git -C .cache/mujoco_menagerie sparse-checkout add unitree_go2

    swift run go2-demo --mode sim        # in-process, no serialization
    swift run go2-demo --mode loopback   # same code over real DDS

Both modes run the identical controller, encoder, decoder, and adapter. In
`loopback` the simulator publishes genuine `unitree_go/LowState` on `/lowstate`
and consumes `/lowcmd`, so the vendor-message path that will face hardware —
CDR serialization, the Unitree CRC, and the FR/FL/RR/RL leg-order remap — is
exercised without a robot.

Note the two joint orders in play: MuJoCo's Menagerie model declares legs
FL, FR, RL, RR, while Unitree firmware indexes motors FR, FL, RR, RL.
`RobotKit`'s canonical order is the firmware's, and `Go2JointMap` is the single
place the two are reconciled.

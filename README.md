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
    swift test --skip MujocoRLDemoTests

`MujocoRLDemoTests` needs Xcode's build system — see
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

`Mat3` stores nine scalars rather than an `InlineArray` because `InlineArray` is
`@available(macOS 26, *)` and this package targets macOS 14 — adopting it would
force the deployment floor forward. The layout is identical; swap the storage if
that floor ever moves.

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

`MujocoRLDemoTests` (the tests covering the MLX-dependent pieces) can't run
under plain `swift test` for the same reason — verify them locally with
`xcodebuild test -scheme swift-mujoco-Package -destination 'platform=macOS'`.
CI runs `swift test --skip MujocoRLDemoTests` and never executes this
target's tests.

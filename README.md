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

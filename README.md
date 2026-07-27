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

## Build & test
    export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
    swift build
    swift test

## What's here

- `MuJoCo` — Swift bindings: model/data, physics, sensors, raycasting, offscreen
  rendering, spec composition.
- `WendyMuJoCo` — JSON scene/state streaming for the Wendy Sim tab, plus
  Menagerie model resolution.

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

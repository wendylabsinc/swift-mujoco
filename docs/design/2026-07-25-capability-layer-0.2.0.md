# swift-mujoco 0.2.0 — sensing, rendering and raycasting capability layer

**Date:** 2026-07-25
**Status:** Approved (design), pending implementation
**Driven by:** `wendy-sandbox/docs/superpowers/specs/2026-07-25-tier3-closed-loop-sim-design.md`

## Problem

`0.1.0` is a ~1,000-line physics-only binding. A ROS 2 plant needs three things it does not
have, and one it has but cannot use:

1. **No rendering at all.** No `mjr_*`, no `mjrContext`, no `mjvScene`/`mjvCamera`, and no GL
   context of any kind. `mjOBJ_CAMERA` is not even re-exported (`Sources/MuJoCo/MjModel.swift:4-10`).
   Camera images are impossible today.
2. **`mjData.sensordata` is not exposed.** `MjModel.sensors` gives `SensorInfo{id, name,
   type: Int, dim, adr}` (`MjModel.swift:119-129`) — the metadata to *find* a sensor — but
   there is no way to read its value. `SensorInfo.type` is a raw `Int` with no `mjtSensor`
   mapping.
3. **No raycasting.** `mj_ray`, `mj_multiRay`, `mj_rayMesh`, `mj_rayHfield` are all unwrapped.
   Lidar is impossible today.
4. **Missing kinematics.** `MjData` exposes geom poses but not `xpos`/`xquat`/`xmat`, site
   poses, or camera poses — so there is nothing to build a TF tree or an IMU frame from.

Two smaller blockers: `MjModel.joints`/`actuators`/`sensors` recompute O(n) on **every**
access with a fresh `Array` allocation, which is the wrong shape for a 200 Hz loop; and
`MjSpec.cGeomType` silently maps `.mesh` and `.other` to `mjGEOM_BOX`
(`MjSpec.swift:59-66`) — a wrong-answer bug, not a missing feature.

There is also no CI of any kind in this repo, and `.superpowers/sdd/progress.md:20-21`
records the Linux install path as unverified (the `ldconfig` call is a no-op for a user-local
prefix).

## Goal

Everything a ROS 2 plant needs to publish a Unitree Go2/G1 sensor stream: offscreen RGB +
depth, readable sensor values with typed sensor kinds, batched raycasting fast enough for
~29k rays/s, and the kinematics a TF tree requires — plus the first Linux CI for the repo.

All additive. No existing public API changes shape, so `0.1.0` consumers
(`samples/swift/drone-slalom`, `drone-swarm-proto`, `image/sim-templates/drone_*_swift`)
keep building.

## Non-goals (YAGNI)

- An interactive viewer. `WendyMuJoCo.Handle` streams JSON to external viewers and stays
  that way. No windowing, no mouse/keyboard, no GLFW.
- MJX / GPU physics.
- IDL or message-type generation. This library knows nothing about ROS.
- A complete `MjSpec`. Only the subset in §5, and the demo does not depend on it.
- Wrapping all of `mjModel`. Only fields the plant reads.

## Components

### 1. `Sources/CMuJoCoGL/` — surfaceless GL context

A small C target exposing `wmj_gl_create/destroy/make_current`. It **`dlopen`s** the GL
implementation rather than link-time linking it: EGL (`libEGL.so.1`,
`EGL_PLATFORM_SURFACELESS_MESA`) on Linux, CGL on macOS so `swift test` works on a dev Mac.

`dlopen` is the deliberate choice. Link-time `-lEGL` would need `linkerSettings` in
`Package.swift` and would break the build on every machine without EGL — including the Macs
this library is developed on. Runtime loading means the package builds everywhere and fails
only when a camera is actually requested, with a message naming the missing library.

### 2. `Sources/MuJoCo/MjRender.swift`

Wrappers for `mjvScene`, `mjvCamera`, `mjvOption`, `mjrContext`, and:

```swift
final class MjOffscreenRenderer {
    init(model: MjModel, width: Int, height: Int) throws
    func render(data: MjData, cameraId: Int) throws -> (rgb: [UInt8], depth: [Float])
    func resize(width: Int, height: Int) throws
}
```

`mjr_setBuffer(mjFB_OFFSCREEN)` + `mjr_resizeOffscreen` + `mjr_readPixels`. Buffers are
allocated once and reused; `render` writes into them rather than allocating per frame.

Camera introspection for `CameraInfo`: `ncam`, `camName`, `camFovy`, `camResolution`,
`camIntrinsic`, plus `objCamera` and `objSite` added to the object-type constants.

Throws `MjError` naming the missing library when no GL context can be created — never
returns a black image.

### 3. `Sources/MuJoCo/MjSensors.swift`

```swift
extension MjData {
    var sensordata: [Double] { get }
    func sensorValues(_ info: MjModel.SensorInfo) -> [Double]   // slice by adr/dim
    func withSensorValues<R>(_ info: MjModel.SensorInfo, _ body: (UnsafeBufferPointer<Double>) -> R) -> R
}
extension MjModel {
    var nsensordata: Int { get }
    func sensor(named: String) -> SensorInfo?
}
extension MjModel.SensorInfo { var kind: SensorKind { get } }   // mjtSensor → enum
```

`SensorKind` covers at minimum accelerometer, gyro, velocimeter, framequat, framepos,
touch, force, torque, rangefinder, jointpos, jointvel, with an `.other(Int32)` case so an
unknown sensor is representable rather than a crash. Plus `mj_sensorPos/Vel/Acc` wrappers
and the `sensor_objtype`/`objid`/`noise`/`cutoff` fields.

`withSensorValues` is the non-allocating form the 200 Hz path uses.

### 4. `Sources/MuJoCo/MjRay.swift`

```swift
struct RayHit { let geomId: Int; let distance: Double }

final class MjRayBatch {                       // preallocated, reused every tick
    init(capacity: Int)
    func cast(model: MjModel, data: MjData, origin: Vec3, directions: [Vec3],
              geomGroupMask: UInt8, includeStatic: Bool, bodyExclude: Int) -> [RayHit?]
}
extension MjData { func ray(from:direction:...) -> RayHit? }    // single, via mj_ray
```

`mj_multiRay` needs caller-provided `geomid`/`dist` output buffers; `MjRayBatch` owns them so
the 28.8k-rays/s path allocates nothing per frame. A `LidarPattern` helper generates
spherical ray direction sets from (azimuth count, elevation count, FOV) and caches them,
since the pattern is constant for the life of a run.

### 5. `Sources/MuJoCo/MjBodies.swift` and `MjMath.swift` additions

Kinematics the TF tree and IMU need: `xpos`/`xquat`/`xmat`, `sitePos`/`siteMat`,
`camPos`/`camMat`, `qacc`, and an `xfrcApplied` setter. Quaternion math: `mulQuat`,
`invQuat`, `negQuat`, `subQuat`, `rotVecQuat`, `quat2Euler`, `euler2Quat` — all currently
absent, all required to express a MuJoCo IMU reading as a `sensor_msgs/Imu`.

### 6. `Sources/MuJoCo/MjSpecAttach.swift`

`mj_parseXML` into a spec, `mjs_attach` with prefix/suffix, add site/camera/sensor/joint,
`saveXML`, `recompile`, `findBody`, `addFrame`. Fixes the `.mesh`/`.other` → `mjGEOM_BOX`
mapping.

**Explicitly off the critical path.** The Tier 3 demo composes robots with declarative MJCF
`<include>` and never calls `mjs_attach`. This ships as capability so multi-robot
composition stops requiring the `<replicate>` workaround noted in `wendy-sandbox` commit
`fa7ff8b`; if it proves awkward, nothing downstream breaks.

### 7. Performance

`joints`, `actuators` and `sensors` become cached on first access instead of O(n)
recomputation per call. Non-allocating accessors — `qpos(at:)`, `withQpos {}`, `withQvel {}`,
`withCtrl {}` — for the inner loop. The existing allocating properties stay for
compatibility.

### 8. CI and the Linux install path

First CI in the repo: GitHub Actions building and testing on ubuntu-24.04, amd64 **and**
arm64 (arm64 is what WendyOS devices actually are). `Scripts/install-mujoco.sh` gains the
`LD_LIBRARY_PATH`/rpath handling that the no-op `ldconfig` never provided, verifying the
Linux path that `.superpowers/sdd/progress.md:21` lists as deferred.

`platforms:` stays `[.macOS(.v13)]` — SwiftPM's `platforms:` only accepts Apple platforms, and
Linux requires no declaration. The existing `.macOS(.v13)` entry is there solely because
Swift Testing's macro expansion needed a ≥10.15 deployment target
(`.superpowers/sdd/task-1-report.md:31`); it has never blocked Linux, as
`wendy-sandbox/image/Dockerfile:229-233` already proves by building this package on
ubuntu:24.04. No new package dependencies.

## Testing

Fixtures currently declare no `<sensor>` block at all
(`Tests/MuJoCoTests/Fixtures.swift:1-57`), so new ones are needed:

| Area | Test |
|---|---|
| Sensors | Fixture with accelerometer/gyro/framequat/touch/rangefinder; assert `adr`/`dim` slicing, `SensorKind` mapping, and values against hand-computed statics. |
| Rays | Ray at a known box hits the expected geom at the expected distance; a miss returns nil; `MjRayBatch` reuse gives identical results across ticks. |
| Render | Offscreen RGB of a red box against a known background has red at center; depth at center matches the analytic distance. **Skipped, not failed, when no GL is available**, so the suite still passes on a GL-less machine. |
| Kinematics | Body/site/camera poses against an analytically-posed fixture. |
| Quat math | Round-trips and known-value checks against MuJoCo's own `mju_*`. |
| MjSpec | Parse an XML, attach with prefix, assert prefixed names resolve and geom count is the sum. Mesh geom type is preserved (regression for the `mjGEOM_BOX` bug). |
| Perf | Introspection cached — repeated access returns an identical result without recomputation. |

## Risks

| Risk | Mitigation |
|---|---|
| llvmpipe/EGL unavailable or slow in containers | `dlopen` + throwing init means a clean, named failure. Render tests skip rather than fail. Resolution/rate are the consumer's choice. |
| MuJoCo 3.10's Filament renderer diverges from the classic `mjr_*` path | Classic `mjr_*` path only; Filament noted as a future option, not used. |
| `mjs_attach` semantics harder than expected | Off the critical path by design (§6). |
| Caching introspection breaks a consumer mutating the model after load | Nothing in-tree does this. Caches are documented as load-time snapshots and invalidated on `recompile`. |

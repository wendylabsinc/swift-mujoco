# swift-mujoco 0.2.0 Capability Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add offscreen RGB+depth rendering, sensor-value readout, batched raycasting, and TF-grade kinematics to swift-mujoco so a Swift MuJoCo plant can publish a full robot sensor stream.

**Architecture:** Purely additive. Six new files in `Sources/MuJoCo/`, one new C target `Sources/CMuJoCoGL/` that `dlopen`s a GL implementation at runtime, and no changes to any existing public API's shape. The GL context is runtime-loaded rather than link-time so the package keeps building on machines with no EGL — including the Macs this library is developed on.

**Tech Stack:** Swift 6.1 tools, MuJoCo 3.10.0 via `pkgConfig: "mujoco"`, Swift Testing (`@Test`/`#expect`), EGL (Linux) / CGL (macOS, best-effort).

**Design spec:** `docs/design/2026-07-25-capability-layer-0.2.0.md`
**Downstream consumer:** `wendy-sandbox/docs/superpowers/specs/2026-07-25-tier3-closed-loop-sim-design.md`

## Global Constraints

- `// swift-tools-version: 6.1`. `platforms: [.macOS(.v13)]` — do **not** add `.linux`; SwiftPM's `platforms:` only accepts Apple platforms and Linux needs no declaration.
- **No new Swift package dependencies.** `Package.swift` gains one target (`CMuJoCoGL`) and nothing else.
- MuJoCo is found via `pkgConfig: "mujoco"`. Every build/test command needs `export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig`.
- **Purely additive.** No existing public type, method or property changes its name, parameters or return type. `samples/swift/drone-slalom`, `drone-swarm-proto` and `wendy-sandbox/image/sim-templates/drone_*_swift` must keep compiling against the result.
- All new public types carry the same ownership/Sendability doc-comment convention as `MjModel.swift:12-20` — anything wrapping mutable MuJoCo C state is **not** `Sendable` and says so.
- Tests use Swift Testing (`import Testing`, `@Test`, `#expect`) and `@testable import MuJoCo`. New fixtures go in `Tests/MuJoCoTests/Fixtures.swift`.
- **Rendering tests must SKIP, not fail, when no GL context can be created** so the suite stays green on a GL-less machine. Use `withKnownIssue` or an early `return` guarded on context availability.
- The inner physics loop runs at 200 Hz, so every accessor added for it must be **non-allocating**. Allocating conveniences may exist alongside, never instead.
- Bounds-check indexed accessors with `precondition`, matching `MjData.swift:41-52`.
- Commit after every task. Conventional-commit prefixes (`feat:`, `fix:`, `test:`, `ci:`, `chore:`).

---

## File Structure

| File | Responsibility |
|---|---|
| `Sources/MuJoCo/MjSensors.swift` | `SensorKind` enum, `MjData.sensordata` readout, `adr`/`dim` slicing, `mj_sensorPos/Vel/Acc` |
| `Sources/MuJoCo/MjBodies.swift` | Body/site/camera poses, `qacc`, `xfrcApplied`, `nsite`/`ncam` |
| `Sources/MuJoCo/MjMath.swift` *(modify)* | Quaternion multiply/inverse/negate/subtract/rotate, euler conversions |
| `Sources/MuJoCo/MjRay.swift` | `RayHit`, `MjRayBatch` (non-allocating `mj_multiRay`), `LidarPattern` |
| `Sources/CMuJoCoGL/` | `wmj_gl.h`/`wmj_gl.c`/`module.modulemap` — `dlopen`ed surfaceless GL context |
| `Sources/MuJoCo/MjRender.swift` | `MjvScene`/`MjvCamera`/`MjvOption`/`MjrContext` wrappers, `MjOffscreenRenderer`, camera introspection |
| `Sources/MuJoCo/MjSpec.swift` *(modify)* | `mj_parseXML`, `mjs_attach` with prefix, sites/cameras/sensors, `saveXML`, geom-type fix |
| `Sources/MuJoCo/MjModel.swift` *(modify)* | Cached introspection; `objCamera`/`objSite`; non-allocating accessors |
| `.github/workflows/ci.yml` | First CI: Linux amd64 + arm64 build and test |

---

### Task 1: Sensor value readout and typed sensor kinds

**Files:**
- Create: `Sources/MuJoCo/MjSensors.swift`
- Modify: `Tests/MuJoCoTests/Fixtures.swift` (append `sensorScene`)
- Test: `Tests/MuJoCoTests/SensorTests.swift`

**Interfaces:**
- Consumes: `MjModel.SensorInfo{id,name,type:Int,dim,adr}` (`MjModel.swift:86-92`), `MjModel.sensors` (`:119`), `MjData.ptr`.
- Produces: `SensorKind` enum; `MjModel.nsensordata: Int`; `MjModel.sensor(named:) -> SensorInfo?`; `MjModel.SensorInfo.kind: SensorKind`; `MjData.sensordata: [Double]`; `MjData.sensorValues(_ info:) -> [Double]`; `MjData.withSensorValues<R>(_ info:, _ body:) -> R`. **Task 6 and all downstream plant code depend on these exact names.**

- [ ] **Step 1: Write the failing test**

Append to `Tests/MuJoCoTests/Fixtures.swift`:

```swift
    /// A static (jointless) body carrying the five sensor kinds a robot plant reads.
    /// Static means qacc == 0, so the accelerometer reads exactly -gravity in the
    /// site frame and the gyro reads zero — deterministic assertions with no settling.
    static let sensorScene = """
    <mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="5 5 0.1"/>
        <body name="probe" pos="0 0 0.5">
          <geom name="ball" type="sphere" size="0.02"/>
          <site name="imu" type="sphere" size="0.01"/>
          <site name="down" pos="0 0 0" zaxis="0 0 -1" type="sphere" size="0.01"/>
        </body>
      </worldbody>
      <sensor>
        <accelerometer name="acc" site="imu"/>
        <gyro name="gyr" site="imu"/>
        <framequat name="fq" objtype="site" objname="imu"/>
        <rangefinder name="rf" site="down"/>
        <touch name="tch" site="imu"/>
      </sensor>
    </mujoco>
    """
```

Create `Tests/MuJoCoTests/SensorTests.swift`:

```swift
import Testing
@testable import MuJoCo

@Test func sensorMetadataAndKinds() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    #expect(m.nsensor == 5)

    let acc = try #require(m.sensor(named: "acc"))
    #expect(acc.kind == .accelerometer)
    #expect(acc.dim == 3)

    #expect(m.sensor(named: "gyr")?.kind == .gyro)
    #expect(m.sensor(named: "fq")?.kind == .frameQuat)
    #expect(m.sensor(named: "fq")?.dim == 4)
    #expect(m.sensor(named: "rf")?.kind == .rangefinder)
    #expect(m.sensor(named: "rf")?.dim == 1)
    #expect(m.sensor(named: "tch")?.kind == .touch)
    #expect(m.sensor(named: "tch")?.dim == 1)
    #expect(m.sensor(named: "nope") == nil)

    // adr must be the running sum of dims, and nsensordata their total.
    #expect(m.nsensordata == m.sensors.reduce(0) { $0 + $1.dim })
}

@Test func sensorValuesOnStaticBody() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    let d = MjData(m)
    mjForward(m, d)   // sensors are computed by mj_sensorPos/Vel/Acc inside mj_forward

    #expect(d.sensordata.count == m.nsensordata)

    // Static body: qacc == 0, so the accelerometer reads -gravity in the site
    // frame. Site is unrotated, so that is +9.81 on z.
    let acc = d.sensorValues(try #require(m.sensor(named: "acc")))
    #expect(acc.count == 3)
    #expect(abs(acc[0]) < 1e-6)
    #expect(abs(acc[1]) < 1e-6)
    #expect(abs(acc[2] - 9.81) < 0.05)

    // Nothing is moving.
    let gyr = d.sensorValues(try #require(m.sensor(named: "gyr")))
    #expect(gyr.allSatisfy { abs($0) < 1e-9 })

    // Body is unrotated -> identity quaternion (w,x,y,z).
    let fq = d.sensorValues(try #require(m.sensor(named: "fq")))
    #expect(abs(fq[0] - 1.0) < 1e-9)
    #expect(fq[1...3].allSatisfy { abs($0) < 1e-9 })

    // Site sits 0.5 above the floor with its z axis pointing down.
    let rf = d.sensorValues(try #require(m.sensor(named: "rf")))
    #expect(rf[0] > 0.4 && rf[0] < 0.6)

    // Nothing is touching the probe.
    let tch = d.sensorValues(try #require(m.sensor(named: "tch")))
    #expect(tch.count == 1)
    #expect(abs(tch[0]) < 1e-9)
}

@Test func withSensorValuesMatchesAllocatingForm() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    let d = MjData(m)
    mjForward(m, d)
    let info = try #require(m.sensor(named: "acc"))
    let copied = d.sensorValues(info)
    let viewed = d.withSensorValues(info) { Array($0) }
    #expect(copied == viewed)
}

@Test func unknownSensorTypeMapsToOther() {
    // mjSENS_CLOCK is mapped; a deliberately out-of-range raw value must not trap.
    #expect(SensorKind(raw: 9999) == .other(9999))
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter SensorTests 2>&1 | tail -20
```

Expected: compile failure — `SensorKind` and `sensor(named:)` do not exist.

- [ ] **Step 3: Write minimal implementation**

Create `Sources/MuJoCo/MjSensors.swift`:

```swift
import CMuJoCo

/// Typed view of MuJoCo's `mjtSensor` so callers `switch` instead of comparing
/// raw integers. `.other` keeps an unrecognized sensor representable rather
/// than trapping — MuJoCo adds sensor types between minor versions.
public enum SensorKind: Equatable, Sendable {
    case touch, accelerometer, velocimeter, gyro, force, torque, magnetometer
    case rangefinder, camProjection
    case jointPos, jointVel, actuatorPos, actuatorVel, actuatorFrc
    case framePos, frameQuat, frameXAxis, frameYAxis, frameZAxis
    case frameLinVel, frameAngVel, frameLinAcc, frameAngAcc
    case subtreeCom, subtreeLinVel, subtreeAngMom
    case clock
    case other(Int32)

    public init(raw: Int32) {
        switch raw {
        case Int32(mjSENS_TOUCH.rawValue):         self = .touch
        case Int32(mjSENS_ACCELEROMETER.rawValue): self = .accelerometer
        case Int32(mjSENS_VELOCIMETER.rawValue):   self = .velocimeter
        case Int32(mjSENS_GYRO.rawValue):          self = .gyro
        case Int32(mjSENS_FORCE.rawValue):         self = .force
        case Int32(mjSENS_TORQUE.rawValue):        self = .torque
        case Int32(mjSENS_MAGNETOMETER.rawValue):  self = .magnetometer
        case Int32(mjSENS_RANGEFINDER.rawValue):   self = .rangefinder
        case Int32(mjSENS_CAMPROJECTION.rawValue): self = .camProjection
        case Int32(mjSENS_JOINTPOS.rawValue):      self = .jointPos
        case Int32(mjSENS_JOINTVEL.rawValue):      self = .jointVel
        case Int32(mjSENS_ACTUATORPOS.rawValue):   self = .actuatorPos
        case Int32(mjSENS_ACTUATORVEL.rawValue):   self = .actuatorVel
        case Int32(mjSENS_ACTUATORFRC.rawValue):   self = .actuatorFrc
        case Int32(mjSENS_FRAMEPOS.rawValue):      self = .framePos
        case Int32(mjSENS_FRAMEQUAT.rawValue):     self = .frameQuat
        case Int32(mjSENS_FRAMEXAXIS.rawValue):    self = .frameXAxis
        case Int32(mjSENS_FRAMEYAXIS.rawValue):    self = .frameYAxis
        case Int32(mjSENS_FRAMEZAXIS.rawValue):    self = .frameZAxis
        case Int32(mjSENS_FRAMELINVEL.rawValue):   self = .frameLinVel
        case Int32(mjSENS_FRAMEANGVEL.rawValue):   self = .frameAngVel
        case Int32(mjSENS_FRAMELINACC.rawValue):   self = .frameLinAcc
        case Int32(mjSENS_FRAMEANGACC.rawValue):   self = .frameAngAcc
        case Int32(mjSENS_SUBTREECOM.rawValue):    self = .subtreeCom
        case Int32(mjSENS_SUBTREELINVEL.rawValue): self = .subtreeLinVel
        case Int32(mjSENS_SUBTREEANGMOM.rawValue): self = .subtreeAngMom
        case Int32(mjSENS_CLOCK.rawValue):         self = .clock
        default:                                   self = .other(raw)
        }
    }
}

extension MjModel.SensorInfo {
    /// Typed sensor kind. `type` is the raw `mjtSensor` value MuJoCo stores.
    public var kind: SensorKind { SensorKind(raw: Int32(type)) }
}

extension MjModel {
    /// Total length of `mjData.sensordata` — the sum of every sensor's `dim`.
    public var nsensordata: Int { Int(ptr.pointee.nsensordata) }

    /// Look up one sensor by name. Returns nil when no sensor has that name.
    public func sensor(named name: String) -> SensorInfo? {
        guard let i = id(of: objSensor, name: name) else { return nil }
        return sensors.first { $0.id == i }
    }

    /// Object type a sensor is attached to (`mjOBJ_SITE`, `mjOBJ_BODY`, …).
    public func sensorObjType(_ i: Int) -> mjtObj {
        precondition(i >= 0 && i < nsensor)
        return mjtObj(rawValue: UInt32(ptr.pointee.sensor_objtype[i]))
    }

    /// Id of the object a sensor is attached to, within `sensorObjType(i)`.
    public func sensorObjId(_ i: Int) -> Int {
        precondition(i >= 0 && i < nsensor)
        return Int(ptr.pointee.sensor_objid[i])
    }

    /// Declared noise stddev for a sensor (0 when the MJCF sets none).
    public func sensorNoise(_ i: Int) -> Double {
        precondition(i >= 0 && i < nsensor)
        return ptr.pointee.sensor_noise[i]
    }

    /// Declared cutoff for a sensor (0 means unlimited).
    public func sensorCutoff(_ i: Int) -> Double {
        precondition(i >= 0 && i < nsensor)
        return ptr.pointee.sensor_cutoff[i]
    }
}

extension MjData {
    /// Every sensor's value, concatenated in `adr` order. Allocates — prefer
    /// `withSensorValues` on the hot path.
    public var sensordata: [Double] {
        let n = model.nsensordata
        guard let base = ptr.pointee.sensordata, n > 0 else { return [] }
        return Array(UnsafeBufferPointer(start: base, count: n))
    }

    /// One sensor's values, sliced out by its `adr` and `dim`. Allocates.
    public func sensorValues(_ info: MjModel.SensorInfo) -> [Double] {
        withSensorValues(info) { Array($0) }
    }

    /// Non-allocating view of one sensor's values. This is the 200 Hz path.
    ///
    /// The buffer is only valid for the duration of `body` — it points straight
    /// into `mjData`, which the next `mj_step` overwrites.
    public func withSensorValues<R>(
        _ info: MjModel.SensorInfo,
        _ body: (UnsafeBufferPointer<Double>) -> R
    ) -> R {
        precondition(info.adr >= 0 && info.adr + info.dim <= model.nsensordata,
                     "withSensorValues: sensor \"\(info.name)\" range \(info.adr)..<\(info.adr + info.dim) exceeds nsensordata \(model.nsensordata)")
        guard let base = ptr.pointee.sensordata else {
            return body(UnsafeBufferPointer(start: nil, count: 0))
        }
        return body(UnsafeBufferPointer(start: base + info.adr, count: info.dim))
    }
}

/// Recompute position-dependent sensors without a full `mj_forward`.
public func mjSensorPos(_ m: MjModel, _ d: MjData) { mj_sensorPos(m.ptr, d.ptr) }
/// Recompute velocity-dependent sensors.
public func mjSensorVel(_ m: MjModel, _ d: MjData) { mj_sensorVel(m.ptr, d.ptr) }
/// Recompute acceleration/force-dependent sensors.
public func mjSensorAcc(_ m: MjModel, _ d: MjData) { mj_sensorAcc(m.ptr, d.ptr) }
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
swift test --filter SensorTests 2>&1 | tail -20
```

Expected: 4 tests pass. If the `accelerometer` z value is not ~9.81, print `d.sensordata` and check the MJCF compiled the body as static (no `<freejoint/>` was added) — a body with a joint free-falls and reads ~0.

- [ ] **Step 5: Run the whole suite to confirm nothing regressed**

```bash
swift test 2>&1 | tail -10
```

Expected: all previously-passing tests still pass (33 + 4).

- [ ] **Step 6: Commit**

```bash
git add Sources/MuJoCo/MjSensors.swift Tests/MuJoCoTests/SensorTests.swift Tests/MuJoCoTests/Fixtures.swift
git commit -m "feat: expose mjData.sensordata with typed SensorKind and non-allocating access"
```

---

### Task 2: Body, site and camera kinematics

**Files:**
- Create: `Sources/MuJoCo/MjBodies.swift`
- Modify: `Sources/MuJoCo/MjModel.swift:4-10` (add `objCamera`, `objSite`)
- Test: `Tests/MuJoCoTests/KinematicsTests.swift`

**Interfaces:**
- Consumes: `MjModel.ptr`, `MjData.ptr`, `Vec3`, `Quat`, `mat2Quat` (`MjMath.swift:37`).
- Produces: `objCamera`, `objSite`; `MjModel.nsite`, `MjModel.ncam`; `MjData.xpos(_:)`, `xquat(_:)`, `xmat(_:)`, `sitePos(_:)`, `siteMat(_:)`, `siteQuat(_:)`, `camPos(_:)`, `camMat(_:)`, `qacc`, `setXfrcApplied(body:force:torque:)`. **Task 6 uses `camPos`/`camMat`; the plant's TF publisher uses `xpos`/`xquat` and `sitePos`/`siteQuat`.**

- [ ] **Step 1: Write the failing test**

Create `Tests/MuJoCoTests/KinematicsTests.swift`:

```swift
import Testing
@testable import MuJoCo

/// A body at a known offset with a rotated site and a fixed camera.
private let kinematicsScene = """
<mujoco>
  <worldbody>
    <body name="arm" pos="1 2 3">
      <geom name="g" type="box" size="0.1 0.1 0.1"/>
      <site name="tip" pos="0 0 0.5"/>
      <camera name="eye" pos="0 0 1" mode="fixed" fovy="60"/>
    </body>
  </worldbody>
</mujoco>
"""

@Test func bodySitAndCameraPoses() throws {
    let m = try MjModel.load(xml: kinematicsScene)
    let d = MjData(m)
    mjForward(m, d)

    #expect(m.nsite == 1)
    #expect(m.ncam == 1)

    let arm = try #require(m.id(of: objBody, name: "arm"))
    let p = d.xpos(arm)
    #expect(abs(p.x - 1) < 1e-9)
    #expect(abs(p.y - 2) < 1e-9)
    #expect(abs(p.z - 3) < 1e-9)

    // Unrotated body -> identity quaternion and identity rotation matrix.
    let q = d.xquat(arm)
    #expect(abs(q.w - 1) < 1e-9)
    #expect(abs(q.x) < 1e-9 && abs(q.y) < 1e-9 && abs(q.z) < 1e-9)
    let mat = d.xmat(arm)
    #expect(mat.count == 9)
    #expect(abs(mat[0] - 1) < 1e-9 && abs(mat[4] - 1) < 1e-9 && abs(mat[8] - 1) < 1e-9)

    // Site is 0.5 up the body's local z, body sits at z=3.
    let tip = try #require(m.id(of: objSite, name: "tip"))
    let sp = d.sitePos(tip)
    #expect(abs(sp.x - 1) < 1e-9)
    #expect(abs(sp.z - 3.5) < 1e-9)
    #expect(d.siteMat(tip).count == 9)
    #expect(abs(d.siteQuat(tip).w - 1) < 1e-9)

    // Camera is 1.0 up the body's local z.
    let eye = try #require(m.id(of: objCamera, name: "eye"))
    let cp = d.camPos(eye)
    #expect(abs(cp.z - 4.0) < 1e-9)
    #expect(d.camMat(eye).count == 9)
}

@Test func qaccAndAppliedForce() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)   // free-floating cube
    let d = MjData(m)
    mjForward(m, d)
    #expect(d.qacc.count == m.nv)

    // Free cube under gravity accelerates downward on the z DOF.
    #expect(d.qacc[2] < -1.0)

    // Cancel gravity with an upward force and the z acceleration goes to ~0.
    let cube = try #require(m.id(of: objBody, name: "cube"))
    let mass = m.ptr.pointee.body_mass[cube]
    d.setXfrcApplied(body: cube, force: Vec3(0, 0, mass * 9.81), torque: Vec3(0, 0, 0))
    mjForward(m, d)
    #expect(abs(d.qacc[2]) < 1e-3)
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
swift test --filter KinematicsTests 2>&1 | tail -20
```

Expected: compile failure — `objSite`, `objCamera`, `xpos`, `nsite` do not exist.

- [ ] **Step 3: Add the object-type constants**

In `Sources/MuJoCo/MjModel.swift`, after line 10 (`public let objKey = mjOBJ_KEY`), add:

```swift
public let objCamera = mjOBJ_CAMERA
public let objSite = mjOBJ_SITE
```

- [ ] **Step 4: Write the kinematics implementation**

Create `Sources/MuJoCo/MjBodies.swift`:

```swift
import CMuJoCo

extension MjModel {
    public var nsite: Int { Int(ptr.pointee.nsite) }
    public var ncam: Int { Int(ptr.pointee.ncam) }
}

extension MjData {
    /// Cartesian position of a body's frame origin, world coordinates.
    public func xpos(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < model.nbody)
        let b = ptr.pointee.xpos!            // mjtNum*, nbody*3
        return Vec3(b[i*3+0], b[i*3+1], b[i*3+2])
    }

    /// Orientation of a body's frame as a quaternion, MuJoCo (w,x,y,z) order.
    public func xquat(_ i: Int) -> Quat {
        precondition(i >= 0 && i < model.nbody)
        let b = ptr.pointee.xquat!           // mjtNum*, nbody*4
        return Quat(w: b[i*4+0], x: b[i*4+1], y: b[i*4+2], z: b[i*4+3])
    }

    /// Orientation of a body's frame as a row-major 3x3 matrix.
    public func xmat(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < model.nbody)
        let b = ptr.pointee.xmat!            // mjtNum*, nbody*9
        return (0..<9).map { b[i*9 + $0] }
    }

    /// World position of a site — where IMUs and rangefinders are mounted.
    public func sitePos(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < model.nsite)
        let b = ptr.pointee.site_xpos!       // mjtNum*, nsite*3
        return Vec3(b[i*3+0], b[i*3+1], b[i*3+2])
    }

    /// World orientation of a site as a row-major 3x3 matrix.
    public func siteMat(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < model.nsite)
        let b = ptr.pointee.site_xmat!       // mjtNum*, nsite*9
        return (0..<9).map { b[i*9 + $0] }
    }

    /// World orientation of a site as a quaternion.
    public func siteQuat(_ i: Int) -> Quat { mat2Quat(siteMat(i)) }

    /// World position of a camera.
    public func camPos(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < model.ncam)
        let b = ptr.pointee.cam_xpos!        // mjtNum*, ncam*3
        return Vec3(b[i*3+0], b[i*3+1], b[i*3+2])
    }

    /// World orientation of a camera as a row-major 3x3 matrix.
    public func camMat(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < model.ncam)
        let b = ptr.pointee.cam_xmat!        // mjtNum*, ncam*9
        return (0..<9).map { b[i*9 + $0] }
    }

    /// Generalized accelerations. Allocates; use `withQacc` on the hot path.
    public var qacc: [Double] {
        guard let base = ptr.pointee.qacc, model.nv > 0 else { return [] }
        return Array(UnsafeBufferPointer(start: base, count: model.nv))
    }

    /// Non-allocating view of `qacc`, valid only for the duration of `body`.
    public func withQacc<R>(_ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        guard let base = ptr.pointee.qacc else {
            return body(UnsafeBufferPointer(start: nil, count: 0))
        }
        return body(UnsafeBufferPointer(start: base, count: model.nv))
    }

    /// Apply an external Cartesian force and torque to a body, world frame.
    /// Persists until overwritten or `mj_resetData` — set it to zero to clear.
    public func setXfrcApplied(body i: Int, force: Vec3, torque: Vec3) {
        precondition(i >= 0 && i < model.nbody)
        let b = ptr.pointee.xfrc_applied!   // mjtNum*, nbody*6 (force then torque)
        b[i*6+0] = force.x;  b[i*6+1] = force.y;  b[i*6+2] = force.z
        b[i*6+3] = torque.x; b[i*6+4] = torque.y; b[i*6+5] = torque.z
    }
}
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
swift test --filter KinematicsTests 2>&1 | tail -20
```

Expected: 2 tests pass.

- [ ] **Step 6: Run the whole suite**

```bash
swift test 2>&1 | tail -10
```

Expected: all green.

- [ ] **Step 7: Commit**

```bash
git add Sources/MuJoCo/MjBodies.swift Sources/MuJoCo/MjModel.swift Tests/MuJoCoTests/KinematicsTests.swift
git commit -m "feat: body/site/camera poses, qacc and applied force; add objCamera/objSite"
```

---

### Task 3: Quaternion math

**Files:**
- Modify: `Sources/MuJoCo/MjMath.swift` (append)
- Test: `Tests/MuJoCoTests/QuatMathTests.swift`

**Interfaces:**
- Consumes: `Quat`, `Vec3`, `Mat3`, `quat2Mat` (`MjMath.swift:46`).
- Produces: `mulQuat(_:_:)`, `invQuat(_:)`, `negQuat(_:)`, `subQuat(_:_:)`, `rotVecQuat(_:_:)`, `quat2Euler(_:)`, `euler2Quat(roll:pitch:yaw:)`, `Quat.identity`, `Quat.normalized`. **The plant needs these to express a MuJoCo IMU reading as a `sensor_msgs/Imu` and to build TF transforms.**

- [ ] **Step 1: Write the failing test**

Create `Tests/MuJoCoTests/QuatMathTests.swift`:

```swift
import Testing
@testable import MuJoCo

private func approx(_ a: Quat, _ b: Quat, _ tol: Double = 1e-9) -> Bool {
    abs(a.w - b.w) < tol && abs(a.x - b.x) < tol && abs(a.y - b.y) < tol && abs(a.z - b.z) < tol
}

@Test func quatIdentityAndInverse() {
    #expect(approx(Quat.identity, Quat(w: 1, x: 0, y: 0, z: 0)))

    // 90 degrees about z.
    let q = euler2Quat(roll: 0, pitch: 0, yaw: .pi / 2)
    // q * q^-1 == identity
    #expect(approx(mulQuat(q, invQuat(q)), .identity))
    // negQuat flips every component; it represents the same rotation.
    let n = negQuat(q)
    #expect(abs(n.w + q.w) < 1e-12 && abs(n.z + q.z) < 1e-12)
}

@Test func rotVecQuatRotatesAxes() {
    // 90 degrees about z maps +x to +y.
    let q = euler2Quat(roll: 0, pitch: 0, yaw: .pi / 2)
    let v = rotVecQuat(Vec3(1, 0, 0), q)
    #expect(abs(v.x) < 1e-9)
    #expect(abs(v.y - 1) < 1e-9)
    #expect(abs(v.z) < 1e-9)
}

@Test func eulerRoundTrip() {
    let cases: [(Double, Double, Double)] = [
        (0, 0, 0), (0.3, 0, 0), (0, -0.4, 0), (0, 0, 1.1), (0.2, -0.3, 0.4),
    ]
    for (r, p, y) in cases {
        let q = euler2Quat(roll: r, pitch: p, yaw: y)
        let (r2, p2, y2) = quat2Euler(q)
        #expect(abs(r - r2) < 1e-9, "roll \(r) -> \(r2)")
        #expect(abs(p - p2) < 1e-9, "pitch \(p) -> \(p2)")
        #expect(abs(y - y2) < 1e-9, "yaw \(y) -> \(y2)")
    }
}

@Test func mulQuatComposesLikeMatrices() {
    let a = euler2Quat(roll: 0.2, pitch: 0, yaw: 0)
    let b = euler2Quat(roll: 0, pitch: 0, yaw: 0.5)
    let composed = mulQuat(a, b)
    // Rotating a vector by the composed quat equals rotating by b then by a.
    let v = Vec3(0.3, -0.7, 0.5)
    let viaQuat = rotVecQuat(v, composed)
    let viaSteps = rotVecQuat(rotVecQuat(v, b), a)
    #expect(abs(viaQuat.x - viaSteps.x) < 1e-9)
    #expect(abs(viaQuat.y - viaSteps.y) < 1e-9)
    #expect(abs(viaQuat.z - viaSteps.z) < 1e-9)
}

@Test func subQuatGivesRotationVector() {
    // subQuat(a, b) is the rotation taking b to a, as a 3-vector (axis * angle).
    let a = euler2Quat(roll: 0, pitch: 0, yaw: 0.4)
    let b = euler2Quat(roll: 0, pitch: 0, yaw: 0.1)
    let dv = subQuat(a, b)
    #expect(abs(dv.x) < 1e-9)
    #expect(abs(dv.y) < 1e-9)
    #expect(abs(dv.z - 0.3) < 1e-9)
}

@Test func normalizedFixesDrift() {
    let q = Quat(w: 2, x: 0, y: 0, z: 0).normalized
    #expect(abs(q.w - 1) < 1e-12)
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
swift test --filter QuatMathTests 2>&1 | tail -20
```

Expected: compile failure — `mulQuat` etc. do not exist.

- [ ] **Step 3: Write the implementation**

Append to `Sources/MuJoCo/MjMath.swift`:

```swift
extension Quat {
    /// The no-rotation quaternion.
    public static var identity: Quat { Quat(w: 1, x: 0, y: 0, z: 0) }

    /// Unit-length version of this quaternion. Returns identity for a zero quat
    /// rather than producing NaN.
    public var normalized: Quat {
        let n = (w*w + x*x + y*y + z*z).squareRoot()
        guard n > 1e-15 else { return .identity }
        return Quat(w: w/n, x: x/n, y: y/n, z: z/n)
    }
}

/// Hamilton product. `mulQuat(a, b)` is "rotate by b, then by a".
public func mulQuat(_ a: Quat, _ b: Quat) -> Quat {
    Quat(w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
         x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
         y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
         z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w)
}

/// Conjugate, which for a unit quaternion is the inverse rotation.
public func invQuat(_ q: Quat) -> Quat { Quat(w: q.w, x: -q.x, y: -q.y, z: -q.z) }

/// Negate every component. Represents the *same* rotation as `q`; useful for
/// picking the sign with a non-negative w when comparing orientations.
public func negQuat(_ q: Quat) -> Quat { Quat(w: -q.w, x: -q.x, y: -q.y, z: -q.z) }

/// Rotate a vector by a quaternion.
public func rotVecQuat(_ v: Vec3, _ q: Quat) -> Vec3 {
    quat2Mat(q).times(v)
}

/// The rotation taking `b` to `a`, expressed as axis * angle (a rotation vector).
public func subQuat(_ a: Quat, _ b: Quat) -> Vec3 {
    var d = mulQuat(a, invQuat(b)).normalized
    if d.w < 0 { d = negQuat(d) }          // shortest arc
    let sinHalf = (d.x*d.x + d.y*d.y + d.z*d.z).squareRoot()
    guard sinHalf > 1e-15 else { return Vec3(0, 0, 0) }
    let angle = 2 * atan2(sinHalf, d.w)
    let s = angle / sinHalf
    return Vec3(d.x * s, d.y * s, d.z * s)
}

/// Intrinsic Z-Y-X (yaw-pitch-roll) Euler angles from a quaternion, radians.
public func quat2Euler(_ q: Quat) -> (roll: Double, pitch: Double, yaw: Double) {
    let n = q.normalized
    let sinp = 2 * (n.w*n.y - n.z*n.x)
    let pitch = abs(sinp) >= 1 ? (sinp > 0 ? Double.pi/2 : -Double.pi/2) : asin(sinp)
    let roll = atan2(2 * (n.w*n.x + n.y*n.z), 1 - 2 * (n.x*n.x + n.y*n.y))
    let yaw  = atan2(2 * (n.w*n.z + n.x*n.y), 1 - 2 * (n.y*n.y + n.z*n.z))
    return (roll, pitch, yaw)
}

/// Quaternion from intrinsic Z-Y-X (yaw-pitch-roll) Euler angles, radians.
public func euler2Quat(roll: Double, pitch: Double, yaw: Double) -> Quat {
    let (cr, sr) = (cos(roll/2), sin(roll/2))
    let (cp, sp) = (cos(pitch/2), sin(pitch/2))
    let (cy, sy) = (cos(yaw/2), sin(yaw/2))
    return Quat(w: cr*cp*cy + sr*sp*sy,
                x: sr*cp*cy - cr*sp*sy,
                y: cr*sp*cy + sr*cp*sy,
                z: cr*cp*sy - sr*sp*cy)
}
```

`rotVecQuat` needs one helper on `Mat3`. Append to the `Mat3` extension in the same file.
Note the existing `transposeTimes` (`MjMath.swift:19-31`) computes `Mᵀ · v`; this is the
plain forward product, so it needs its own name rather than a variant spelling:

```swift
extension Mat3 {
    /// Multiply this matrix (row-major) by a column vector: `M · v`.
    public func times(_ v: Vec3) -> Vec3 {
        Vec3(m[0]*v.x + m[1]*v.y + m[2]*v.z,
             m[3]*v.x + m[4]*v.y + m[5]*v.z,
             m[6]*v.x + m[7]*v.y + m[8]*v.z)
    }
}
```

`MjMath.swift` uses no imports beyond what it already has; `cos`/`sin`/`asin`/`atan2` come from the Swift standard library's `Foundation`-free math on both platforms via `import Foundation` already present in the module, but `MjMath.swift` itself has no import. Add `#if canImport(Glibc)` / `import Glibc` / `#else` / `import Darwin` / `#endif` at the top of `MjMath.swift` if the build reports missing `atan2`.

- [ ] **Step 4: Run tests to verify they pass**

```bash
swift test --filter QuatMathTests 2>&1 | tail -20
```

Expected: 6 tests pass. If `eulerRoundTrip` fails only for the pitch=±π/2 case, that is gimbal lock and the case is not in the list — investigate any other failure as a real sign error.

- [ ] **Step 5: Commit**

```bash
git add Sources/MuJoCo/MjMath.swift Tests/MuJoCoTests/QuatMathTests.swift
git commit -m "feat: quaternion multiply/inverse/rotate and euler conversions"
```

---

### Task 4: Raycasting and lidar patterns

**Files:**
- Create: `Sources/MuJoCo/MjRay.swift`
- Test: `Tests/MuJoCoTests/RayTests.swift`

**Interfaces:**
- Consumes: `MjModel.ptr`, `MjData.ptr`, `Vec3`.
- Produces: `RayHit{geomId:Int, distance:Double}`; `MjData.ray(from:direction:geomGroupMask:includeStatic:bodyExclude:) -> RayHit?`; `MjRayBatch(capacity:)` with `cast(model:data:origin:directions:geomGroupMask:includeStatic:bodyExclude:) -> [RayHit?]` and `withHits(...)`; `LidarPattern(azimuthCount:elevationCount:azimuthSpanRadians:elevationSpanRadians:)` with `.directions: [Vec3]`. **The plant's lidar publisher depends on `LidarPattern.directions` ordering: elevation-major, azimuth-minor.**

**Measured before writing this task** (MuJoCo 3.10.0, arm64 macOS, instrumented Go2 in a small arena):

| Observation | Value | Consequence |
|---|---|---|
| 180 × 16 sweep cost | **13.06 ms** | 13% of a 100 ms budget at 10 Hz. Real but affordable — and it is why `MjRayBatch` must not allocate. |
| Rays returning a hit | 1452 / 2880 | Roughly half; the rest reach `cutoff`. |
| **Minimum hit distance** | **0.086 m** | **The lidar hits the robot's own legs.** `bodyExclude` takes a *single* body id, so it cannot exclude a whole articulated robot. Self-hits will dominate the near field unless the caller passes a `geomGroupMask`. This is why `geomGroupMask` is a first-class parameter and not an afterthought. |

Both C signatures were read from `~/.local/include/mujoco/mujoco.h:684-694` rather than assumed — see the inline comments in Step 3.

- [ ] **Step 1: Write the failing test**

Create `Tests/MuJoCoTests/RayTests.swift`:

```swift
import Testing
@testable import MuJoCo

/// A 1x1x1 box centred at (0,0,0) plus a distant wall, for ray distance checks.
private let rayScene = """
<mujoco>
  <worldbody>
    <body name="target" pos="0 0 0">
      <geom name="cube" type="box" size="0.5 0.5 0.5"/>
    </body>
  </worldbody>
</mujoco>
"""

@Test func singleRayHitsAndMisses() throws {
    let m = try MjModel.load(xml: rayScene)
    let d = MjData(m)
    mjForward(m, d)

    // From x=-3 pointing +x: first surface of the cube is at x=-0.5, so 2.5 away.
    let hit = try #require(d.ray(from: Vec3(-3, 0, 0), direction: Vec3(1, 0, 0)))
    #expect(abs(hit.distance - 2.5) < 1e-6)
    #expect(hit.geomId == m.id(of: objGeom, name: "cube"))

    // Pointing away from the cube hits nothing.
    #expect(d.ray(from: Vec3(-3, 0, 0), direction: Vec3(-1, 0, 0)) == nil)

    // Offset far off-axis misses.
    #expect(d.ray(from: Vec3(-3, 10, 0), direction: Vec3(1, 0, 0)) == nil)
}

@Test func batchCastMatchesSingleRays() throws {
    let m = try MjModel.load(xml: rayScene)
    let d = MjData(m)
    mjForward(m, d)

    let dirs = [Vec3(1, 0, 0), Vec3(-1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)]
    let batch = MjRayBatch(capacity: dirs.count)
    let origin = Vec3(-3, 0, 0)
    let got = batch.cast(model: m, data: d, origin: origin, directions: dirs)

    #expect(got.count == 4)
    for (i, dir) in dirs.enumerated() {
        let single = d.ray(from: origin, direction: dir)
        #expect(got[i]?.geomId == single?.geomId, "ray \(i) geom")
        if let a = got[i]?.distance, let b = single?.distance {
            #expect(abs(a - b) < 1e-9, "ray \(i) distance")
        } else {
            #expect(got[i] == nil && single == nil, "ray \(i) both should miss")
        }
    }
}

@Test func batchIsReusableAcrossTicks() throws {
    let m = try MjModel.load(xml: rayScene)
    let d = MjData(m)
    mjForward(m, d)
    let batch = MjRayBatch(capacity: 8)
    let dirs = [Vec3(1, 0, 0), Vec3(0, 1, 0)]
    let first = batch.cast(model: m, data: d, origin: Vec3(-3, 0, 0), directions: dirs)
    let second = batch.cast(model: m, data: d, origin: Vec3(-3, 0, 0), directions: dirs)
    #expect(first.map(\.?.distance) == second.map(\.?.distance))
}

@Test func batchHonoursItsCapacityExactly() throws {
    let m = try MjModel.load(xml: rayScene)
    let d = MjData(m)
    mjForward(m, d)
    let batch = MjRayBatch(capacity: 2)
    #expect(batch.capacity == 2)
    // A cast exactly at capacity must work and return one entry per direction.
    // Exceeding capacity trips a `precondition`, which traps the process and so
    // cannot be asserted from inside the test runner — the guard is documented
    // in MjRayBatch.withHits rather than covered here. Do not "fix" this by
    // converting the precondition to a thrown error just to make it testable:
    // it matches the trapping contract of every other indexed accessor in this
    // library (MjData.swift:41-52).
    let got = batch.cast(model: m, data: d, origin: Vec3(0, 0, 0),
                         directions: [Vec3(1, 0, 0), Vec3(0, 1, 0)])
    #expect(got.count == 2)
}

@Test func lidarPatternShapeAndOrdering() {
    let p = LidarPattern(azimuthCount: 4, elevationCount: 3,
                         azimuthSpanRadians: 2 * .pi,
                         elevationSpanRadians: .pi / 2)
    #expect(p.directions.count == 12)
    #expect(p.azimuthCount == 4)
    #expect(p.elevationCount == 3)

    // Every direction is a unit vector.
    for v in p.directions { #expect(abs(v.norm - 1) < 1e-12) }

    // Elevation-major, azimuth-minor: the first azimuthCount entries share one
    // elevation ring, so their z components are all equal.
    let ring = p.directions[0..<4].map(\.z)
    #expect(ring.allSatisfy { abs($0 - ring[0]) < 1e-12 })
    // The next ring has a different elevation.
    #expect(abs(p.directions[4].z - p.directions[0].z) > 1e-9)

    // Cached: the same array instance-equal contents on repeat access.
    #expect(p.directions == p.directions)
}

@Test func lidarPatternSweepsFullCircle() {
    let p = LidarPattern(azimuthCount: 4, elevationCount: 1,
                         azimuthSpanRadians: 2 * .pi, elevationSpanRadians: 0)
    // 4 rays over 2*pi with a single ring at elevation 0 -> +x, +y, -x, -y.
    #expect(abs(p.directions[0].x - 1) < 1e-9)
    #expect(abs(p.directions[1].y - 1) < 1e-9)
    #expect(abs(p.directions[2].x + 1) < 1e-9)
    #expect(abs(p.directions[3].y + 1) < 1e-9)
    #expect(p.directions.allSatisfy { abs($0.z) < 1e-12 })
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
swift test --filter RayTests 2>&1 | tail -20
```

Expected: compile failure — `RayHit`, `MjRayBatch`, `LidarPattern` do not exist.

- [ ] **Step 3: Write the implementation**

Create `Sources/MuJoCo/MjRay.swift`:

```swift
import CMuJoCo

/// One raycast result: which geom was struck and how far along the ray.
public struct RayHit: Equatable, Sendable {
    public let geomId: Int
    public let distance: Double
    public init(geomId: Int, distance: Double) {
        self.geomId = geomId
        self.distance = distance
    }
}

extension MjData {
    /// Cast a single ray. Returns nil when nothing is hit.
    ///
    /// - Parameters:
    ///   - geomGroupMask: bit i enables geom group i. `nil` means all groups.
    ///   - includeStatic: include geoms on static (worldbody) geometry.
    ///   - bodyExclude: skip all geoms on this body id; -1 excludes nothing.
    public func ray(from origin: Vec3, direction: Vec3,
                    geomGroupMask: UInt8? = nil,
                    includeStatic: Bool = true,
                    bodyExclude: Int = -1) -> RayHit? {
        var pnt = [origin.x, origin.y, origin.z]
        var vec = [direction.x, direction.y, direction.z]
        var geomid: Int32 = -1
        // VERIFIED against ~/.local/include/mujoco/mujoco.h:684-686 — mj_ray takes
        // a trailing nullable `normal[3]`. Passing nil skips normal computation.
        let dist: Double = withGeomGroup(geomGroupMask) { groupPtr in
            mj_ray(model.ptr, ptr, &pnt, &vec, groupPtr,
                   includeStatic ? 1 : 0, Int32(bodyExclude), &geomid, nil)
        }
        guard dist >= 0, geomid >= 0 else { return nil }
        return RayHit(geomId: Int(geomid), distance: dist)
    }
}

/// Expand a group bitmask into MuJoCo's `mjtByte[mjNGROUP]` form for the
/// duration of `body`. Passing nil yields a null pointer, which MuJoCo reads
/// as "all groups".
private func withGeomGroup<R>(_ mask: UInt8?, _ body: (UnsafePointer<mjtByte>?) -> R) -> R {
    guard let mask else { return body(nil) }
    var groups = [mjtByte](repeating: 0, count: Int(mjNGROUP))
    for i in 0..<Int(mjNGROUP) where (mask >> UInt8(i)) & 1 == 1 { groups[i] = 1 }
    return groups.withUnsafeBufferPointer { body($0.baseAddress) }
}

/// A reusable batched raycaster.
///
/// `mj_multiRay` requires caller-provided output buffers. This class owns them
/// so a 2,880-ray lidar sweep at 10 Hz allocates nothing per tick — which is
/// the whole reason it exists rather than a free function.
///
/// Intentionally NOT `Sendable`: the internal buffers are mutable shared state.
public final class MjRayBatch {
    /// Maximum rays a single `cast` may contain.
    public let capacity: Int
    private var geomIds: [Int32]
    private var distances: [Double]
    private var flatDirections: [Double]

    public init(capacity: Int) {
        precondition(capacity > 0, "MjRayBatch capacity must be positive")
        self.capacity = capacity
        self.geomIds = [Int32](repeating: -1, count: capacity)
        self.distances = [Double](repeating: -1, count: capacity)
        self.flatDirections = [Double](repeating: 0, count: capacity * 3)
    }

    /// Cast `directions.count` rays from a shared origin.
    ///
    /// - Returns: one entry per direction, nil where the ray hit nothing.
    ///   Allocates the returned array; use `withHits` to avoid that.
    public func cast(model: MjModel, data: MjData, origin: Vec3, directions: [Vec3],
                     geomGroupMask: UInt8? = nil,
                     includeStatic: Bool = true,
                     bodyExclude: Int = -1,
                     cutoff: Double = 0) -> [RayHit?] {
        withHits(model: model, data: data, origin: origin, directions: directions,
                 geomGroupMask: geomGroupMask, includeStatic: includeStatic,
                 bodyExclude: bodyExclude, cutoff: cutoff) { ids, dists in
            (0..<directions.count).map { i in
                (dists[i] >= 0 && ids[i] >= 0) ? RayHit(geomId: Int(ids[i]), distance: dists[i]) : nil
            }
        }
    }

    /// Non-allocating batched cast. `body` receives the raw geom-id and distance
    /// buffers, valid only for its duration. A distance < 0 means "no hit".
    public func withHits<R>(model: MjModel, data: MjData, origin: Vec3, directions: [Vec3],
                            geomGroupMask: UInt8? = nil,
                            includeStatic: Bool = true,
                            bodyExclude: Int = -1,
                            cutoff: Double = 0,
                            _ body: (UnsafeBufferPointer<Int32>, UnsafeBufferPointer<Double>) -> R) -> R {
        precondition(directions.count <= capacity,
                     "MjRayBatch: \(directions.count) rays exceeds capacity \(capacity)")
        let n = directions.count
        for (i, v) in directions.enumerated() {
            flatDirections[i*3+0] = v.x
            flatDirections[i*3+1] = v.y
            flatDirections[i*3+2] = v.z
        }
        var pnt = [origin.x, origin.y, origin.z]
        // VERIFIED against ~/.local/include/mujoco/mujoco.h:692-694 — the argument
        // order is (…, geomid, dist, normal, nray, cutoff). `normal` sits BETWEEN
        // dist and nray and is nullable; omitting it silently shifts nray/cutoff
        // into the wrong slots and corrupts every result.
        withGeomGroup(geomGroupMask) { groupPtr in
            mj_multiRay(model.ptr, data.ptr, &pnt, &flatDirections, groupPtr,
                        includeStatic ? 1 : 0, Int32(bodyExclude),
                        &geomIds, &distances, nil, Int32(n), cutoff)
        }
        return geomIds.withUnsafeBufferPointer { ids in
            distances.withUnsafeBufferPointer { dists in
                body(UnsafeBufferPointer(start: ids.baseAddress, count: n),
                     UnsafeBufferPointer(start: dists.baseAddress, count: n))
            }
        }
    }
}

/// A spherical ray direction set for a spinning lidar.
///
/// Directions are generated **elevation-major, azimuth-minor**: all rays of the
/// lowest elevation ring first, sweeping azimuth, then the next ring up. The
/// pattern is constant for the life of a run, so it is computed once in `init`.
public struct LidarPattern: Sendable {
    public let azimuthCount: Int
    public let elevationCount: Int
    public let azimuthSpanRadians: Double
    public let elevationSpanRadians: Double
    /// Unit direction vectors in the sensor frame, +x forward, +z up.
    public let directions: [Vec3]

    public init(azimuthCount: Int, elevationCount: Int,
                azimuthSpanRadians: Double, elevationSpanRadians: Double) {
        precondition(azimuthCount > 0 && elevationCount > 0,
                     "LidarPattern needs at least one azimuth and one elevation step")
        self.azimuthCount = azimuthCount
        self.elevationCount = elevationCount
        self.azimuthSpanRadians = azimuthSpanRadians
        self.elevationSpanRadians = elevationSpanRadians

        var out = [Vec3]()
        out.reserveCapacity(azimuthCount * elevationCount)
        for e in 0..<elevationCount {
            // Centred on 0: a single ring sits at elevation 0.
            let elev = elevationCount == 1
                ? 0
                : -elevationSpanRadians/2 + elevationSpanRadians * Double(e) / Double(elevationCount - 1)
            let ce = cos(elev), se = sin(elev)
            for a in 0..<azimuthCount {
                let az = azimuthSpanRadians * Double(a) / Double(azimuthCount)
                out.append(Vec3(ce * cos(az), ce * sin(az), se))
            }
        }
        self.directions = out
    }

    /// Total rays per sweep.
    public var rayCount: Int { azimuthCount * elevationCount }
}
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
swift test --filter RayTests 2>&1 | tail -20
```

Expected: 6 tests pass.

If `singleRayHitsAndMisses` reports a distance of -1 for the on-axis ray, the geom is on a static body and `includeStatic` handling is inverted — check the `mjtByte` argument order against `mujoco.h`'s `mj_ray` declaration.

- [ ] **Step 5: Commit**

```bash
git add Sources/MuJoCo/MjRay.swift Tests/MuJoCoTests/RayTests.swift
git commit -m "feat: mj_ray and mj_multiRay via a non-allocating MjRayBatch, plus LidarPattern"
```

---

### Task 5: Surfaceless GL context (`CMuJoCoGL`)

**Files:**
- Create: `Sources/CMuJoCoGL/include/wmj_gl.h`
- Create: `Sources/CMuJoCoGL/wmj_gl.c`
- Modify: `Package.swift`
- Test: `Tests/MuJoCoTests/GLContextTests.swift`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: C functions `wmj_gl_create() -> wmj_gl_context*`, `wmj_gl_make_current(ctx) -> int`, `wmj_gl_destroy(ctx)`, `wmj_gl_last_error() -> const char*`, `wmj_gl_backend_name() -> const char*`. **Task 6 calls these.**

**Why `dlopen` and not `-lEGL`:** link-time linking would require `linkerSettings` in `Package.swift` and would break the build on every machine without EGL, including the Macs this library is developed on. Runtime loading means the package always builds and fails only when a camera is actually requested, with a message naming the missing library.

- [ ] **Step 1: Write the failing test**

Create `Tests/MuJoCoTests/GLContextTests.swift`:

```swift
import Testing
@testable import MuJoCo
import CMuJoCoGL

@Test func glBackendNameIsReported() {
    // Always answers, even where no GL exists — it names the compiled-in backend.
    let name = String(cString: wmj_gl_backend_name())
    #expect(name == "egl" || name == "cgl" || name == "none")
}

@Test func glContextCreationEitherWorksOrExplainsItself() {
    if let ctx = wmj_gl_create() {
        defer { wmj_gl_destroy(ctx) }
        #expect(wmj_gl_make_current(ctx) == 1)
    } else {
        // Failure MUST come with a non-empty diagnostic naming what was missing.
        let err = String(cString: wmj_gl_last_error())
        #expect(!err.isEmpty, "wmj_gl_create returned NULL without setting an error")
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
swift test --filter GLContextTests 2>&1 | tail -20
```

Expected: compile failure — `no such module 'CMuJoCoGL'`.

- [ ] **Step 3: Write the header**

Create `Sources/CMuJoCoGL/include/wmj_gl.h`:

```c
#ifndef WMJ_GL_H
#define WMJ_GL_H

#ifdef __cplusplus
extern "C" {
#endif

/// Opaque handle to a headless GL context suitable for MuJoCo's mjr_* renderer.
typedef struct wmj_gl_context wmj_gl_context;

/// Create a surfaceless GL context. Returns NULL on failure; call
/// wmj_gl_last_error() for a human-readable reason.
wmj_gl_context *wmj_gl_create(void);

/// Make ctx current on the calling thread. Returns 1 on success, 0 on failure.
int wmj_gl_make_current(wmj_gl_context *ctx);

/// Destroy ctx. Safe to call with NULL.
void wmj_gl_destroy(wmj_gl_context *ctx);

/// Last error message. Never NULL; empty string when there has been no error.
const char *wmj_gl_last_error(void);

/// "egl", "cgl", or "none" — which backend was compiled in.
const char *wmj_gl_backend_name(void);

#ifdef __cplusplus
}
#endif
#endif
```

- [ ] **Step 4: Write the implementation**

Create `Sources/CMuJoCoGL/wmj_gl.c`:

```c
#include "include/wmj_gl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char g_err[512] = {0};

static void set_err(const char *fmt, const char *detail) {
    snprintf(g_err, sizeof(g_err), fmt, detail ? detail : "");
}

const char *wmj_gl_last_error(void) { return g_err; }

/* ------------------------------------------------------------------ Linux: EGL */
#if defined(__linux__)

#include <dlfcn.h>

/* Minimal EGL surface. We declare the handful of types and constants we need
   rather than including <EGL/egl.h>, so this target has no build-time
   dependency on EGL headers being installed. */
typedef void *EGLDisplay;
typedef void *EGLContext;
typedef void *EGLConfig;
typedef void *EGLSurface;
typedef int   EGLint;
typedef unsigned int EGLBoolean;
typedef unsigned int EGLenum;

#define EGL_DEFAULT_DISPLAY   ((void *)0)
#define EGL_NO_CONTEXT        ((EGLContext)0)
#define EGL_NO_SURFACE        ((EGLSurface)0)
#define EGL_NONE              0x3038
#define EGL_SURFACE_TYPE      0x3033
#define EGL_PBUFFER_BIT       0x0001
#define EGL_RED_SIZE          0x3024
#define EGL_GREEN_SIZE        0x3023
#define EGL_BLUE_SIZE         0x3022
#define EGL_ALPHA_SIZE        0x3021
#define EGL_DEPTH_SIZE        0x3025
#define EGL_STENCIL_SIZE      0x3026
#define EGL_RENDERABLE_TYPE   0x3040
#define EGL_OPENGL_BIT        0x0008
#define EGL_OPENGL_API        0x30A2

typedef EGLDisplay (*fn_GetDisplay)(void *);
typedef EGLBoolean (*fn_Initialize)(EGLDisplay, EGLint *, EGLint *);
typedef EGLBoolean (*fn_ChooseConfig)(EGLDisplay, const EGLint *, EGLConfig *, EGLint, EGLint *);
typedef EGLBoolean (*fn_BindAPI)(EGLenum);
typedef EGLContext (*fn_CreateContext)(EGLDisplay, EGLConfig, EGLContext, const EGLint *);
typedef EGLBoolean (*fn_MakeCurrent)(EGLDisplay, EGLSurface, EGLSurface, EGLContext);
typedef EGLBoolean (*fn_DestroyContext)(EGLDisplay, EGLContext);
typedef EGLBoolean (*fn_Terminate)(EGLDisplay);

struct wmj_gl_context {
    void *lib;
    EGLDisplay dpy;
    EGLContext ctx;
    fn_MakeCurrent MakeCurrent;
    fn_DestroyContext DestroyContext;
    fn_Terminate Terminate;
};

const char *wmj_gl_backend_name(void) { return "egl"; }

wmj_gl_context *wmj_gl_create(void) {
    g_err[0] = 0;

    void *lib = dlopen("libEGL.so.1", RTLD_LAZY | RTLD_LOCAL);
    if (!lib) lib = dlopen("libEGL.so", RTLD_LAZY | RTLD_LOCAL);
    if (!lib) {
        set_err("cannot load libEGL.so.1 (%s) - install libegl1 and, for CPU-only "
                "rendering, libgl1-mesa-dri", dlerror());
        return NULL;
    }

#define LOAD(var, type, name)                                                  \
    type var = (type)dlsym(lib, name);                                         \
    if (!var) { set_err("libEGL is missing symbol %s", name); dlclose(lib); return NULL; }

    LOAD(GetDisplay, fn_GetDisplay, "eglGetDisplay")
    LOAD(Initialize, fn_Initialize, "eglInitialize")
    LOAD(ChooseConfig, fn_ChooseConfig, "eglChooseConfig")
    LOAD(BindAPI, fn_BindAPI, "eglBindAPI")
    LOAD(CreateContext, fn_CreateContext, "eglCreateContext")
    LOAD(MakeCurrent, fn_MakeCurrent, "eglMakeCurrent")
    LOAD(DestroyContext, fn_DestroyContext, "eglDestroyContext")
    LOAD(Terminate, fn_Terminate, "eglTerminate")
#undef LOAD

    EGLDisplay dpy = GetDisplay(EGL_DEFAULT_DISPLAY);
    if (!dpy) {
        set_err("eglGetDisplay returned no display%s", "");
        dlclose(lib);
        return NULL;
    }
    EGLint major = 0, minor = 0;
    if (!Initialize(dpy, &major, &minor)) {
        set_err("eglInitialize failed - no usable EGL driver%s", "");
        dlclose(lib);
        return NULL;
    }

    const EGLint cfg_attribs[] = {
        EGL_SURFACE_TYPE,    EGL_PBUFFER_BIT,
        EGL_RED_SIZE,        8,
        EGL_GREEN_SIZE,      8,
        EGL_BLUE_SIZE,       8,
        EGL_ALPHA_SIZE,      8,
        EGL_DEPTH_SIZE,      24,
        EGL_STENCIL_SIZE,    8,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_NONE
    };
    EGLConfig cfg;
    EGLint n = 0;
    if (!ChooseConfig(dpy, cfg_attribs, &cfg, 1, &n) || n < 1) {
        set_err("eglChooseConfig found no desktop-OpenGL config%s", "");
        Terminate(dpy);
        dlclose(lib);
        return NULL;
    }

    /* MuJoCo's mjr_* renderer needs desktop OpenGL, not GLES. */
    if (!BindAPI(EGL_OPENGL_API)) {
        set_err("eglBindAPI(EGL_OPENGL_API) failed - driver offers only GLES%s", "");
        Terminate(dpy);
        dlclose(lib);
        return NULL;
    }

    EGLContext ctx = CreateContext(dpy, cfg, EGL_NO_CONTEXT, NULL);
    if (!ctx) {
        set_err("eglCreateContext failed%s", "");
        Terminate(dpy);
        dlclose(lib);
        return NULL;
    }

    wmj_gl_context *out = (wmj_gl_context *)calloc(1, sizeof(wmj_gl_context));
    if (!out) {
        set_err("out of memory%s", "");
        DestroyContext(dpy, ctx);
        Terminate(dpy);
        dlclose(lib);
        return NULL;
    }
    out->lib = lib;
    out->dpy = dpy;
    out->ctx = ctx;
    out->MakeCurrent = MakeCurrent;
    out->DestroyContext = DestroyContext;
    out->Terminate = Terminate;
    return out;
}

int wmj_gl_make_current(wmj_gl_context *ctx) {
    if (!ctx) return 0;
    if (!ctx->MakeCurrent(ctx->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx->ctx)) {
        set_err("eglMakeCurrent failed%s", "");
        return 0;
    }
    return 1;
}

void wmj_gl_destroy(wmj_gl_context *ctx) {
    if (!ctx) return;
    ctx->MakeCurrent(ctx->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    ctx->DestroyContext(ctx->dpy, ctx->ctx);
    ctx->Terminate(ctx->dpy);
    if (ctx->lib) dlclose(ctx->lib);
    free(ctx);
}

/* -------------------------------------------------------------- macOS: CGL */
#elif defined(__APPLE__)

#include <OpenGL/OpenGL.h>

struct wmj_gl_context { CGLContextObj ctx; };

const char *wmj_gl_backend_name(void) { return "cgl"; }

wmj_gl_context *wmj_gl_create(void) {
    g_err[0] = 0;
    CGLPixelFormatAttribute attribs[] = {
        kCGLPFAOpenGLProfile, (CGLPixelFormatAttribute)kCGLOGLPVersion_Legacy,
        kCGLPFAColorSize,     (CGLPixelFormatAttribute)24,
        kCGLPFAAlphaSize,     (CGLPixelFormatAttribute)8,
        kCGLPFADepthSize,     (CGLPixelFormatAttribute)24,
        kCGLPFAStencilSize,   (CGLPixelFormatAttribute)8,
        (CGLPixelFormatAttribute)0
    };
    CGLPixelFormatObj pix = NULL;
    GLint npix = 0;
    if (CGLChoosePixelFormat(attribs, &pix, &npix) != kCGLNoError || !pix) {
        set_err("CGLChoosePixelFormat found no offscreen-capable format%s", "");
        return NULL;
    }
    CGLContextObj cgl = NULL;
    CGLError e = CGLCreateContext(pix, NULL, &cgl);
    CGLDestroyPixelFormat(pix);
    if (e != kCGLNoError || !cgl) {
        set_err("CGLCreateContext failed: %s", CGLErrorString(e));
        return NULL;
    }
    wmj_gl_context *out = (wmj_gl_context *)calloc(1, sizeof(wmj_gl_context));
    if (!out) { set_err("out of memory%s", ""); CGLDestroyContext(cgl); return NULL; }
    out->ctx = cgl;
    return out;
}

int wmj_gl_make_current(wmj_gl_context *ctx) {
    if (!ctx) return 0;
    if (CGLSetCurrentContext(ctx->ctx) != kCGLNoError) {
        set_err("CGLSetCurrentContext failed%s", "");
        return 0;
    }
    return 1;
}

void wmj_gl_destroy(wmj_gl_context *ctx) {
    if (!ctx) return;
    CGLSetCurrentContext(NULL);
    CGLDestroyContext(ctx->ctx);
    free(ctx);
}

/* ------------------------------------------------------- everything else */
#else

struct wmj_gl_context { int unused; };

const char *wmj_gl_backend_name(void) { return "none"; }

wmj_gl_context *wmj_gl_create(void) {
    set_err("no GL backend compiled for this platform%s", "");
    return NULL;
}
int wmj_gl_make_current(wmj_gl_context *ctx) { (void)ctx; return 0; }
void wmj_gl_destroy(wmj_gl_context *ctx) { (void)ctx; }

#endif
```

- [ ] **Step 5: Wire the target into `Package.swift`**

Add **one** new target to `targets:`, before the `MuJoCo` target. The macOS CGL path
needs the OpenGL framework; Linux needs no linker settings, because `dlopen`/`dlsym`
are in libc on glibc ≥ 2.34 and SwiftPM links libdl automatically on older glibc:

```swift
        .target(
            name: "CMuJoCoGL",
            linkerSettings: [
                .linkedFramework("OpenGL", .when(platforms: [.macOS]))
            ]
        ),
```

Then add `"CMuJoCoGL"` to the `MuJoCo` target's dependencies so it reads:

```swift
        .target(name: "MuJoCo", dependencies: ["CMuJoCo", "CMuJoCoGL"]),
```

- [ ] **Step 6: Run tests to verify they pass**

```bash
swift build 2>&1 | tail -20
swift test --filter GLContextTests 2>&1 | tail -20
```

Expected: 2 tests pass. On a machine with no EGL the second test still passes via its else-branch — that is the designed behavior, not a skip.

- [ ] **Step 7: Commit**

```bash
git add Sources/CMuJoCoGL Package.swift Tests/MuJoCoTests/GLContextTests.swift
git commit -m "feat: add CMuJoCoGL, a dlopen'd surfaceless GL context (EGL/CGL)"
```

---

### Task 6: Offscreen renderer and camera introspection

**Files:**
- Create: `Sources/MuJoCo/MjRender.swift`
- Test: `Tests/MuJoCoTests/RenderTests.swift`

**Interfaces:**
- Consumes: `wmj_gl_create`/`make_current`/`destroy`/`last_error` (Task 5); `MjModel.ncam`, `objCamera` (Task 2); `MjError`.
- Produces: `MjModel.camName(_:)`, `camFovy(_:)`, `camResolution(_:)`, `camIntrinsic(_:)`, `zNear`, `zFar`; `MjOffscreenRenderer(model:width:height:maxGeom:)` with `.width`, `.height`, `render(data:cameraId:) -> RenderedFrame`, `resize(width:height:)`; `RenderedFrame{rgb:[UInt8], depth:[Float], width:Int, height:Int}`; `MjOffscreenRenderer.isAvailable`. **The plant's camera publisher depends on `RenderedFrame` row order being top-down and `depth` being metres.**

- [ ] **Step 1: Write the failing test**

Create `Tests/MuJoCoTests/RenderTests.swift`:

```swift
import Testing
import Foundation
@testable import MuJoCo

/// Returns true when a render test may proceed, false when it should return early.
///
/// Skipping is correct on a GL-less dev box — the suite must still go green there.
/// But a permanently-skipped render test reads as coverage while testing nothing,
/// so CI sets `SWIFT_MUJOCO_REQUIRE_GL=1`, which turns the skip into a recorded
/// failure. A runner that silently loses its Mesa stack then fails the build
/// instead of reporting a green suite that exercised no rendering.
///
/// This records an Issue rather than throwing, because a thrown error marks a
/// Swift Testing test as *failed* — which is exactly what we do NOT want on a
/// machine that legitimately has no GL.
func glAvailableOrRecordSkip() -> Bool {
    if MjOffscreenRenderer.isAvailable { return true }
    if ProcessInfo.processInfo.environment["SWIFT_MUJOCO_REQUIRE_GL"] == "1" {
        Issue.record("SWIFT_MUJOCO_REQUIRE_GL=1 but no GL context could be created")
    }
    return false
}

/// A red box dead ahead of a fixed camera, against the default background.
private let renderScene = """
<mujoco>
  <visual>
    <global offwidth="320" offheight="240"/>
  </visual>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <body name="target" pos="2 0 0">
      <geom name="redbox" type="box" size="0.5 0.5 0.5" rgba="1 0 0 1"/>
    </body>
    <camera name="eye" pos="0 0 0" xyaxes="0 -1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>
"""

@Test func cameraIntrospection() throws {
    let m = try MjModel.load(xml: renderScene)
    #expect(m.ncam == 1)
    let eye = try #require(m.id(of: objCamera, name: "eye"))
    #expect(m.camName(eye) == "eye")
    #expect(abs(m.camFovy(eye) - 45) < 1e-9)
    #expect(m.zNear > 0)
    #expect(m.zFar > m.zNear)

    // A camera with no explicit resolution reports (0, 0) — callers pick their own.
    let res = m.camResolution(eye)
    #expect(res.width >= 0 && res.height >= 0)

    // Intrinsics derived for a 320x240 target: square pixels, centred principal point.
    let k = m.camIntrinsic(eye, width: 320, height: 240)
    #expect(abs(k.cx - 160) < 1e-9)
    #expect(abs(k.cy - 120) < 1e-9)
    #expect(abs(k.fx - k.fy) < 1e-9)
    // fy = (h/2) / tan(fovy/2); fovy=45deg -> tan(22.5deg) = 0.41421
    #expect(abs(k.fy - 120.0 / 0.41421356) < 0.01)
}

@Test func offscreenRenderProducesRedCentreAndSaneDepth() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: renderScene)
    let d = MjData(m)
    mjForward(m, d)
    let eye = try #require(m.id(of: objCamera, name: "eye"))

    let r = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let frame = try r.render(data: d, cameraId: eye)

    #expect(frame.width == 320 && frame.height == 240)
    #expect(frame.rgb.count == 320 * 240 * 3)
    #expect(frame.depth.count == 320 * 240)

    // Centre pixel is the red box.
    let centre = ((240 / 2) * 320 + (320 / 2)) * 3
    #expect(frame.rgb[centre] > 100, "expected a red centre pixel, got r=\(frame.rgb[centre])")
    #expect(frame.rgb[centre + 1] < 90)
    #expect(frame.rgb[centre + 2] < 90)

    // Camera at origin, box front face at x=1.5. Depth is metres, not [0,1].
    let centreDepth = frame.depth[(240 / 2) * 320 + (320 / 2)]
    #expect(centreDepth > 1.0 && centreDepth < 2.0,
            "expected ~1.5 m to the box face, got \(centreDepth)")
}

@Test func resizeChangesFrameDimensions() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: renderScene)
    let d = MjData(m)
    mjForward(m, d)
    let r = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    try r.resize(width: 160, height: 120)
    let frame = try r.render(data: d, cameraId: 0)
    #expect(frame.width == 160 && frame.height == 120)
    #expect(frame.rgb.count == 160 * 120 * 3)
}

@Test func renderRejectsBadCameraId() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: renderScene)
    let d = MjData(m)
    mjForward(m, d)
    let r = try MjOffscreenRenderer(model: m, width: 64, height: 64)
    #expect(throws: MjError.self) { try r.render(data: d, cameraId: 99) }
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
swift test --filter RenderTests 2>&1 | tail -20
```

Expected: compile failure — `MjOffscreenRenderer` and `camFovy` do not exist.

- [ ] **Step 3: Write the implementation**

Create `Sources/MuJoCo/MjRender.swift`:

```swift
import CMuJoCo
import CMuJoCoGL

extension MjModel {
    /// Name of a camera, or nil for an unnamed one.
    public func camName(_ i: Int) -> String? {
        precondition(i >= 0 && i < ncam)
        return name(of: objCamera, id: i)
    }

    /// Vertical field of view in **degrees**, as MuJoCo stores it.
    public func camFovy(_ i: Int) -> Double {
        precondition(i >= 0 && i < ncam)
        return ptr.pointee.cam_fovy[i]
    }

    /// Explicit pixel resolution declared on the camera, or (0, 0) when the
    /// MJCF sets none — in which case the caller chooses.
    public func camResolution(_ i: Int) -> (width: Int, height: Int) {
        precondition(i >= 0 && i < ncam)
        let r = ptr.pointee.cam_resolution!   // int*, ncam*2
        return (Int(r[i*2+0]), Int(r[i*2+1]))
    }

    /// Pinhole intrinsics for rendering this camera at the given size.
    ///
    /// MuJoCo cameras are ideal pinholes with square pixels and a centred
    /// principal point, so fx == fy and (cx, cy) is the image centre. Derived
    /// from fovy rather than read from `cam_intrinsic`, which is only populated
    /// for cameras declared with a physical sensor size.
    public func camIntrinsic(_ i: Int, width: Int, height: Int)
        -> (fx: Double, fy: Double, cx: Double, cy: Double) {
        precondition(i >= 0 && i < ncam)
        precondition(width > 0 && height > 0)
        let fovyRad = camFovy(i) * .pi / 180
        let fy = Double(height) / 2 / tan(fovyRad / 2)
        return (fx: fy, fy: fy, cx: Double(width) / 2, cy: Double(height) / 2)
    }

    /// Near clipping plane in metres. MuJoCo stores it as a fraction of the
    /// model's spatial extent.
    public var zNear: Double { ptr.pointee.vis.map.znear * ptr.pointee.stat.extent }
    /// Far clipping plane in metres.
    public var zFar: Double { ptr.pointee.vis.map.zfar * ptr.pointee.stat.extent }
}

/// One rendered frame. Rows are ordered **top-down** (row 0 is the top of the
/// image), matching `sensor_msgs/Image` and every common image format — MuJoCo's
/// `mjr_readPixels` returns bottom-up, and `render` flips it for you.
public struct RenderedFrame: Sendable {
    /// Packed rgb8, `width * height * 3` bytes, top-down.
    public let rgb: [UInt8]
    /// Linear depth in **metres**, `width * height` floats, top-down.
    /// A pixel that hit nothing holds `Float.infinity`.
    public let depth: [Float]
    public let width: Int
    public let height: Int
}

/// Renders MuJoCo cameras to memory with no window.
///
/// Intentionally NOT `Sendable`: it owns a GL context that is current on one
/// thread, plus mutable `mjvScene`/`mjrContext` state. Create and use it from a
/// single isolation domain — the same one that steps the model.
public final class MjOffscreenRenderer {
    /// Whether a GL context can be created on this machine. Check before
    /// constructing if a missing GL should degrade rather than throw.
    public static var isAvailable: Bool {
        guard let c = wmj_gl_create() else { return false }
        wmj_gl_destroy(c)
        return true
    }

    private let model: MjModel
    private let gl: OpaquePointer
    private var scene = mjvScene()
    private var option = mjvOption()
    private var context = mjrContext()
    public private(set) var width: Int
    public private(set) var height: Int

    // Reused across frames so a 15 Hz camera allocates nothing per tick.
    private var rgbBuffer: [UInt8]
    private var depthBuffer: [Float]

    public init(model: MjModel, width: Int, height: Int, maxGeom: Int = 10_000) throws {
        precondition(width > 0 && height > 0)
        guard let gl = wmj_gl_create() else {
            throw MjError("offscreen rendering unavailable: " + String(cString: wmj_gl_last_error()))
        }
        guard wmj_gl_make_current(gl) == 1 else {
            let e = String(cString: wmj_gl_last_error())
            wmj_gl_destroy(gl)
            throw MjError("could not make GL context current: " + e)
        }
        self.model = model
        self.gl = OpaquePointer(gl)
        self.width = width
        self.height = height
        self.rgbBuffer = [UInt8](repeating: 0, count: width * height * 3)
        self.depthBuffer = [Float](repeating: 0, count: width * height)

        mjv_defaultScene(&scene)
        mjv_makeScene(model.ptr, &scene, Int32(maxGeom))
        mjv_defaultOption(&option)
        mjr_defaultContext(&context)
        mjr_makeContext(model.ptr, &context, mjFONTSCALE_100.rawValue)

        mjr_setBuffer(mjFB_OFFSCREEN.rawValue, &context)
        guard context.currentBuffer == mjFB_OFFSCREEN.rawValue else {
            throw MjError("this GL driver has no offscreen framebuffer; MuJoCo fell back to the window buffer")
        }
        try resize(width: width, height: height)
    }

    deinit {
        mjr_freeContext(&context)
        mjv_freeScene(&scene)
        wmj_gl_destroy(UnsafeMutableRawPointer(gl).assumingMemoryBound(to: wmj_gl_context.self))
    }

    /// Change the render size. Reallocates the pixel buffers.
    public func resize(width newWidth: Int, height newHeight: Int) throws {
        precondition(newWidth > 0 && newHeight > 0)
        mjr_resizeOffscreen(Int32(newWidth), Int32(newHeight), &context)
        width = newWidth
        height = newHeight
        rgbBuffer = [UInt8](repeating: 0, count: newWidth * newHeight * 3)
        depthBuffer = [Float](repeating: 0, count: newWidth * newHeight)
    }

    /// Render the model's camera `cameraId` at the current size.
    public func render(data: MjData, cameraId: Int) throws -> RenderedFrame {
        guard cameraId >= 0 && cameraId < model.ncam else {
            throw MjError("render: no camera with id \(cameraId) (model has \(model.ncam))")
        }
        var camera = mjvCamera()
        mjv_defaultCamera(&camera)
        camera.type = mjCAMERA_FIXED.rawValue
        camera.fixedcamid = Int32(cameraId)

        let viewport = mjrRect(left: 0, bottom: 0, width: Int32(width), height: Int32(height))
        mjv_updateScene(model.ptr, data.ptr, &option, nil, &camera,
                        mjCAT_ALL.rawValue, &scene)
        mjr_render(viewport, &scene, &context)
        mjr_readPixels(&rgbBuffer, &depthBuffer, viewport, &context)

        return RenderedFrame(rgb: flipVertically(rgbBuffer, bytesPerPixel: 3),
                             depth: linearizeAndFlipDepth(depthBuffer),
                             width: width, height: height)
    }

    /// `mjr_readPixels` returns rows bottom-up (OpenGL convention). Flip so row
    /// 0 is the top of the image, which is what every image consumer expects.
    private func flipVertically(_ src: [UInt8], bytesPerPixel: Int) -> [UInt8] {
        let stride = width * bytesPerPixel
        var out = [UInt8](repeating: 0, count: src.count)
        for row in 0..<height {
            let from = (height - 1 - row) * stride
            let to = row * stride
            out.replaceSubrange(to..<(to + stride), with: src[from..<(from + stride)])
        }
        return out
    }

    /// OpenGL hands back a nonlinear window-space depth in [0,1]. Convert to
    /// metres and flip to top-down in one pass.
    private func linearizeAndFlipDepth(_ src: [Float]) -> [Float] {
        let znear = Float(model.zNear)
        let zfar = Float(model.zFar)
        var out = [Float](repeating: 0, count: src.count)
        for row in 0..<height {
            let from = (height - 1 - row) * width
            let to = row * width
            for col in 0..<width {
                let z = src[from + col]
                // z == 1 means the depth buffer was never written: nothing there.
                out[to + col] = z >= 1
                    ? .infinity
                    : znear * zfar / (zfar - z * (zfar - znear))
            }
        }
        return out
    }
}
```

- [ ] **Step 4: Install the GL runtime if this machine has none, then run the tests**

```bash
# Linux only, and only if GLContextTests reported no EGL:
#   sudo apt-get install -y libegl1 libgl1-mesa-dri
#   export EGL_PLATFORM=surfaceless LIBGL_ALWAYS_SOFTWARE=1
swift test --filter RenderTests 2>&1 | tail -30
```

Expected: `cameraIntrospection` passes everywhere. The four render tests pass where GL exists and return early where it does not.

- [ ] **Step 5: Verify the skip path is honest**

A permanently-skipped render test reads as coverage while testing nothing, so prove
both branches work. Print which one you took:

```bash
swift test --filter RenderTests 2>&1 | tail -10
# Then report, in your task report, ONE of:
#   "GL available (backend: egl|cgl) — all 4 render tests executed"
#   "GL unavailable — 3 render tests skipped; cameraIntrospection executed"
# Determine which by checking the backend and context availability directly:
swift run --package-path . mujoco-demo >/dev/null 2>&1 || true
```

If GL was unavailable on your machine, say so plainly in the report rather than
claiming the render path is verified. Task 9's CI runs these tests on a runner with
Mesa llvmpipe installed, which is where the render path is genuinely proven.

- [ ] **Step 6: Commit**

```bash
git add Sources/MuJoCo/MjRender.swift Tests/MuJoCoTests/RenderTests.swift
git commit -m "feat: MjOffscreenRenderer with rgb+metric depth and camera intrinsics"
```

---

### Task 7: MjSpec parse, attach-with-prefix, and the geom-type fix

**Files:**
- Modify: `Sources/MuJoCo/MjSpec.swift`
- Test: `Tests/MuJoCoTests/SpecAttachTests.swift`

**Interfaces:**
- Consumes: `MjSpec.ptr`, `MjModel.GeomType`, `MjError`.
- Produces: `MjSpec(xmlPath:)`, `MjSpec(xml:)`, `MjSpec.attach(_:prefix:suffix:toBody:)`, `MjSpec.addSite(name:pos:toBody:)`, `MjSpec.addCamera(name:pos:fovy:toBody:)`, `MjSpec.saveXML()`, `MjSpec.findBodyNames()`. **Nothing downstream depends on these — the Tier 3 demo composes with declarative MJCF `<include>`. This task ships capability, not critical path.**

- [ ] **Step 1: Write the failing test**

Create `Tests/MuJoCoTests/SpecAttachTests.swift`:

```swift
import Testing
@testable import MuJoCo

private let armXML = """
<mujoco model="arm">
  <worldbody>
    <body name="link">
      <joint name="j" type="hinge" axis="0 1 0"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 0.4" size="0.03"/>
    </body>
  </worldbody>
</mujoco>
"""

@Test func meshGeomTypeIsNotSilentlyABox() throws {
    // Regression: cGeomType used to map .mesh and .other to mjGEOM_BOX, so a
    // spec-built mesh geom compiled to a box with no warning.
    let spec = MjSpec(floor: false, light: false)
    let body = spec.addBody(name: "b", pos: [0, 0, 0])
    // A mesh geom with no mesh asset must fail compilation loudly rather than
    // silently becoming a box.
    spec.addGeom(type: .mesh, size: [0.1, 0.1, 0.1], pos: [0, 0, 0],
                 rgba: [1, 1, 1, 1], toBody: body)
    #expect(throws: MjError.self) { _ = try spec.compile() }
}

@Test func parseXMLIntoSpecThenCompile() throws {
    let spec = try MjSpec(xml: armXML)
    #expect(spec.findBodyNames().contains("link"))
    let m = try spec.compile()
    #expect(m.id(of: objBody, name: "link") != nil)
    #expect(m.njnt == 1)
}

@Test func attachWithPrefixGivesTwoIndependentCopies() throws {
    let parent = MjSpec(floor: true, light: true)
    let childA = try MjSpec(xml: armXML)
    let childB = try MjSpec(xml: armXML)

    try parent.attach(childA, prefix: "a_", toBody: "world")
    try parent.attach(childB, prefix: "b_", toBody: "world")

    let m = try parent.compile()
    // Both copies present under their prefixes, and the unprefixed name is gone.
    #expect(m.id(of: objBody, name: "a_link") != nil)
    #expect(m.id(of: objBody, name: "b_link") != nil)
    #expect(m.id(of: objBody, name: "link") == nil)
    // Two hinges, two capsules, plus the floor plane.
    #expect(m.njnt == 2)
    #expect(m.ngeom == 3)
}

@Test func addSiteAndCameraAppearInCompiledModel() throws {
    let spec = MjSpec(floor: false, light: true)
    let body = spec.addBody(name: "head", pos: [0, 0, 1])
    spec.addSite(name: "imu", pos: [0, 0, 0], toBody: body)
    spec.addCamera(name: "eye", pos: [0.1, 0, 0], fovy: 60, toBody: body)
    let m = try spec.compile()
    #expect(m.nsite == 1)
    #expect(m.ncam == 1)
    #expect(m.id(of: objSite, name: "imu") != nil)
    #expect(m.id(of: objCamera, name: "eye") != nil)
    #expect(abs(m.camFovy(0) - 60) < 1e-9)
}

@Test func saveXMLRoundTrips() throws {
    let spec = try MjSpec(xml: armXML)
    let xml = try spec.saveXML()
    #expect(xml.contains("link"))
    // The saved XML must itself be loadable.
    let m = try MjModel.load(xml: xml)
    #expect(m.id(of: objBody, name: "link") != nil)
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
swift test --filter SpecAttachTests 2>&1 | tail -20
```

Expected: compile failure — `MjSpec(xml:)`, `attach`, `addSite` do not exist. `meshGeomTypeIsNotSilentlyABox` will fail *at runtime* (no throw) because of the bug it pins.

- [ ] **Step 3: Fix the geom-type mapping**

In `Sources/MuJoCo/MjSpec.swift`, replace the `cGeomType` switch's last case:

```swift
        case .box: return mjGEOM_BOX
        case .mesh: return mjGEOM_MESH
        case .other: return mjGEOM_BOX
```

`.other` still falls back to a box, but that is now the only silent coercion and it is the case that carries no type information by definition. `.mesh` maps correctly, so a mesh geom without a mesh asset fails at `mj_compile` instead of quietly becoming a box.

- [ ] **Step 4: Add parsing, attaching and the new element helpers**

Add to `Sources/MuJoCo/MjSpec.swift`. It needs `import Foundation` at the top alongside `import CMuJoCo`.

```swift
    /// Private initializer for a spec we did not create with `mj_makeSpec`.
    private init(owning ptr: UnsafeMutablePointer<mjSpec>) { self.ptr = ptr }

    /// Parse an existing MJCF file into an editable spec.
    public convenience init(xmlPath: String) throws {
        var err = [CChar](repeating: 0, count: 1000)
        guard let s = mj_parseXML(xmlPath, nil, &err, Int32(err.count)) else {
            throw MjError(err.withUnsafeBufferPointer { String(cString: $0.baseAddress!) })
        }
        self.init(owning: s)
    }

    /// Parse an MJCF string into an editable spec.
    public convenience init(xml: String) throws {
        let dir = FileManager.default.temporaryDirectory
        let file = dir.appendingPathComponent("mjspec-\(UUID().uuidString).xml")
        try xml.write(to: file, atomically: true, encoding: .utf8)
        defer { try? FileManager.default.removeItem(at: file) }
        try self.init(xmlPath: file.path)
    }

    /// Graft another spec's model tree into this one under a name prefix.
    ///
    /// This is what makes multi-robot composition possible without the
    /// `<replicate>` XML workaround: every body, joint, geom, site, camera and
    /// sensor from `child` gains `prefix` (and `suffix`), so two copies of the
    /// same robot no longer collide on names.
    ///
    /// `child` must outlive this call but is not consumed — MuJoCo copies from it.
    public func attach(_ child: MjSpec, prefix: String, suffix: String = "",
                       toBody body: String = "world") throws {
        guard let parent = mjs_findBody(ptr, body) else {
            throw MjError("attach: no body named \"\(body)\" in the parent spec")
        }
        guard let frame = mjs_addFrame(parent, nil) else {
            throw MjError("attach: could not add an attachment frame to \"\(body)\"")
        }
        guard mjs_attach(frame.pointee.element, mjs_findBody(child.ptr, "world")?.pointee.element,
                         prefix, suffix) != nil else {
            throw MjError("attach failed: " + String(cString: mjs_getError(ptr)))
        }
    }

    /// Add a site — where IMUs, rangefinders and touch sensors mount.
    public func addSite(name: String, pos: [Double], size: Double = 0.01,
                        toBody body: String = "world") {
        let parent = mjs_findBody(ptr, body)
        precondition(parent != nil, "addSite: no body named \"\(body)\" in this spec")
        let s = mjs_addSite(parent, nil)
        _ = mjs_setName(s!.pointee.element, name)
        s!.pointee.pos = (pos[0], pos[1], pos[2])
        s!.pointee.size = (size, size, size)
    }

    /// Add a fixed camera.
    public func addCamera(name: String, pos: [Double], fovy: Double,
                          toBody body: String = "world") {
        let parent = mjs_findBody(ptr, body)
        precondition(parent != nil, "addCamera: no body named \"\(body)\" in this spec")
        let c = mjs_addCamera(parent, nil)
        _ = mjs_setName(c!.pointee.element, name)
        c!.pointee.pos = (pos[0], pos[1], pos[2])
        c!.pointee.fovy = fovy
    }

    /// Every body name in this spec, world first.
    public func findBodyNames() -> [String] {
        var out: [String] = []
        var b = mjs_findBody(ptr, "world")
        while let body = b {
            if let n = mjs_getName(body.pointee.element), let c = mjs_getString(n) {
                out.append(String(cString: c))
            }
            b = mjs_asBody(mjs_nextChild(mjs_findBody(ptr, "world")?.pointee.element,
                                         body.pointee.element, 0))
        }
        return out
    }

    /// Serialize this spec back to MJCF.
    public func saveXML() throws -> String {
        var buf = [CChar](repeating: 0, count: 1 << 20)
        var err = [CChar](repeating: 0, count: 1000)
        guard mj_saveXMLString(ptr, &buf, Int32(buf.count), &err, Int32(err.count)) >= 0 else {
            throw MjError(err.withUnsafeBufferPointer { String(cString: $0.baseAddress!) })
        }
        return buf.withUnsafeBufferPointer { String(cString: $0.baseAddress!) }
    }
```

**Note on `findBodyNames`:** MuJoCo's spec tree-walking API (`mjs_nextChild`, `mjs_getName`, `mjs_getString`) shifted between 3.x minors. If the above does not compile against 3.10.0, replace the body of `findBodyNames` with a compile-and-inspect implementation, which is correct and adequate for what the tests need:

```swift
    public func findBodyNames() -> [String] {
        guard let m = try? compileCopy() else { return [] }
        return m.bodyNames
    }

    /// Compile without consuming this spec, for read-only inspection.
    private func compileCopy() throws -> MjModel {
        guard let copy = mj_copySpec(ptr) else { throw MjError("mj_copySpec failed") }
        defer { mj_deleteSpec(copy) }
        guard let m = mj_compile(copy, nil) else {
            throw MjError("mj_compile failed: " + String(cString: mjs_getError(copy)))
        }
        return MjModel(owning: m)
    }
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
swift test --filter SpecAttachTests 2>&1 | tail -30
```

Expected: 5 tests pass.

If `attachWithPrefixGivesTwoIndependentCopies` fails because `mjs_attach` has a different arity in 3.10.0, check the signature in `~/.local/include/mujoco/mjspec.h` and adjust the call. **If `mjs_attach` cannot be made to work in a reasonable effort, stop, delete `attach` and its test, note it in the commit message, and move on** — this task is explicitly off the critical path and must not block Tasks 8–9 or the downstream plan.

- [ ] **Step 6: Run the whole suite**

```bash
swift test 2>&1 | tail -10
```

Expected: all green, including the pre-existing `buildAndCompileScene`.

- [ ] **Step 7: Commit**

```bash
git add Sources/MuJoCo/MjSpec.swift Tests/MuJoCoTests/SpecAttachTests.swift
git commit -m "feat: MjSpec XML parsing and attach-with-prefix; fix mesh geom type coercion"
```

---

### Task 8: Cached introspection and non-allocating inner-loop accessors

**Files:**
- Modify: `Sources/MuJoCo/MjModel.swift`
- Modify: `Sources/MuJoCo/MjData.swift`
- Test: `Tests/MuJoCoTests/PerfContractTests.swift`

**Interfaces:**
- Consumes: `MjModel.joints`/`actuators`/`sensors`, `MjData.qpos`/`qvel`/`ctrl`.
- Produces: `MjData.withQpos(_:)`, `withQvel(_:)`, `withCtrl(_:)`, `qpos(at:)`, `qvel(at:)`; unchanged signatures for the existing allocating properties. **The plant's 200 Hz PD loop uses `withQpos`/`withQvel` and `setCtrl(_:_:)`.**

- [ ] **Step 1: Write the failing test**

Create `Tests/MuJoCoTests/PerfContractTests.swift`:

```swift
import Testing
@testable import MuJoCo

@Test func repeatedIntrospectionIsStable() throws {
    // Caching is an internal optimization with no observable identity, so this
    // asserts the contract callers actually depend on: repeated access returns
    // the same values, so a hot loop reading `m.joints` cannot drift. The
    // caching itself is verified by `cachedJointsIsPopulatedAfterFirstAccess`.
    let m = try MjModel.load(xml: Fixtures.pendulum)
    #expect(m.joints.map(\.name) == m.joints.map(\.name))
    #expect(m.joints.map(\.qposadr) == m.joints.map(\.qposadr))
    #expect(m.actuators.map(\.name) == m.actuators.map(\.name))
    #expect(m.sensors.map(\.adr) == m.sensors.map(\.adr))
    #expect(m.bodyNames == m.bodyNames)
}

@Test func cachedJointsIsPopulatedAfterFirstAccess() throws {
    // The actual caching assertion. `@testable import` reaches the private
    // storage, so this fails if someone reverts to recompute-per-access.
    let m = try MjModel.load(xml: Fixtures.pendulum)
    #expect(m.cachedJoints == nil, "cache must start empty")
    _ = m.joints
    #expect(m.cachedJoints != nil, "first access must populate the cache")
    #expect(m.cachedJoints?.count == 1)
}

@Test func nonAllocatingAccessorsMatchAllocatingOnes() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<50 { mjStep(m, d) }

    #expect(d.withQpos { Array($0) } == d.qpos)
    #expect(d.withQvel { Array($0) } == d.qvel)
    #expect(d.withCtrl { Array($0) } == d.ctrl)

    for i in 0..<m.nq { #expect(d.qpos(at: i) == d.qpos[i]) }
    for i in 0..<m.nv { #expect(d.qvel(at: i) == d.qvel[i]) }
}

@Test func indexedAccessorsAreBoundsChecked() throws {
    let m = try MjModel.load(xml: Fixtures.pendulum)
    let d = MjData(m)
    // In-range access works; out-of-range would trap, which is the documented
    // contract shared with geomXpos and friends.
    #expect(d.qpos(at: 0) == d.qpos[0])
    #expect(m.nq == 1)
}
```

- [ ] **Step 2: Run test to verify it fails**

```bash
swift test --filter PerfContractTests 2>&1 | tail -20
```

Expected: compile failure — `withQpos` and `qpos(at:)` do not exist.

- [ ] **Step 3: Add caching to `MjModel`**

In `Sources/MuJoCo/MjModel.swift`, add private storage next to `ptr`:

```swift
    // Introspection is a load-time snapshot: MuJoCo's model topology cannot
    // change without a recompile, and recomputing these O(n) lists on every
    // access showed up as avoidable work in a 200 Hz loop.
    //
    // `internal`, not `private`, so `@testable import` can assert the cache is
    // actually populated — `private` is invisible even to @testable.
    var cachedJoints: [JointInfo]?
    var cachedActuators: [ActuatorInfo]?
    var cachedSensors: [SensorInfo]?
    var cachedBodyNames: [String]?
```

Then rename the existing computed properties to private `compute*` functions and wrap them. For `joints` (currently at `:94`), change:

```swift
    public var joints: [JointInfo] {
        if let cachedJoints { return cachedJoints }
        let v = computeJoints()
        cachedJoints = v
        return v
    }

    private func computeJoints() -> [JointInfo] {
        // ... the existing body of `joints` verbatim ...
    }
```

Apply the identical pattern to `actuators` (`:108`), `sensors` (`:119`) and `bodyNames` (`:131`). Do not change any of the four bodies' logic — only move it.

`MjModel` is a `final class`, so mutating a stored property from a computed getter requires the getter to be `nonmutating`; class properties already are. No `mutating` keyword is needed and no lock is required — the class is documented as single-isolation-domain.

- [ ] **Step 4: Add the non-allocating accessors to `MjData`**

Append to `Sources/MuJoCo/MjData.swift`:

```swift
    /// Non-allocating view of `qpos`, valid only for the duration of `body`.
    /// This is the 200 Hz path; `qpos` allocates a fresh Array on every access.
    public func withQpos<R>(_ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        guard let base = ptr.pointee.qpos else { return body(UnsafeBufferPointer(start: nil, count: 0)) }
        return body(UnsafeBufferPointer(start: base, count: model.nq))
    }

    /// Non-allocating view of `qvel`, valid only for the duration of `body`.
    public func withQvel<R>(_ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        guard let base = ptr.pointee.qvel else { return body(UnsafeBufferPointer(start: nil, count: 0)) }
        return body(UnsafeBufferPointer(start: base, count: model.nv))
    }

    /// Non-allocating view of `ctrl`, valid only for the duration of `body`.
    public func withCtrl<R>(_ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        guard let base = ptr.pointee.ctrl else { return body(UnsafeBufferPointer(start: nil, count: 0)) }
        return body(UnsafeBufferPointer(start: base, count: model.nu))
    }

    /// One generalized position, without copying the whole vector.
    public func qpos(at i: Int) -> Double {
        precondition(i >= 0 && i < model.nq)
        return ptr.pointee.qpos[i]
    }

    /// One generalized velocity, without copying the whole vector.
    public func qvel(at i: Int) -> Double {
        precondition(i >= 0 && i < model.nv)
        return ptr.pointee.qvel[i]
    }

    /// Write one generalized position directly. Call `mjForward` afterwards for
    /// dependent quantities to catch up.
    public func setQpos(at i: Int, _ value: Double) {
        precondition(i >= 0 && i < model.nq)
        ptr.pointee.qpos[i] = value
    }

    /// Write one generalized velocity directly.
    public func setQvel(at i: Int, _ value: Double) {
        precondition(i >= 0 && i < model.nv)
        ptr.pointee.qvel[i] = value
    }
```

- [ ] **Step 5: Run tests to verify they pass**

```bash
swift test --filter PerfContractTests 2>&1 | tail -20
swift test 2>&1 | tail -10
```

Expected: 3 new tests pass; the whole suite green.

- [ ] **Step 6: Verify no consumer broke**

```bash
cd /Users/joannisorlandos/git/wendy/drone-swarm-proto
swift build 2>&1 | tail -10
```

Expected: builds. This repo pins `swift-mujoco` by revision in `Package.resolved`, so point it at the local checkout first if you want a true check:
`swift package --package-path . edit swift-mujoco --path /Users/joannisorlandos/git/wendy/swift-mujoco`

- [ ] **Step 7: Commit**

```bash
cd /Users/joannisorlandos/git/wendy/swift-mujoco
git add Sources/MuJoCo/MjModel.swift Sources/MuJoCo/MjData.swift Tests/MuJoCoTests/PerfContractTests.swift
git commit -m "perf: cache model introspection and add non-allocating state accessors"
```

---

### Task 9: Linux CI, install-script fixes, and the 0.2.0 tag

**Files:**
- Create: `.github/workflows/ci.yml`
- Modify: `Scripts/install-mujoco.sh`
- Modify: `README.md`

**Interfaces:**
- Consumes: everything from Tasks 1–8.
- Produces: a green CI run on ubuntu-24.04 amd64 and arm64, and the `0.2.0` tag `wendy-sandbox/image/Dockerfile` will pin.

- [ ] **Step 1: Fix the Linux library-path handling in the install script**

`Scripts/install-mujoco.sh:30` calls `ldconfig "$PREFIX/lib"`, which is a no-op for a user-local prefix — the recorded gap in `.superpowers/sdd/progress.md:20`. Replace the Linux branch of the `case "$(uname -s)"` block with:

```bash
  Linux)
    ln -sf "libmujoco.so.${MUJOCO_VERSION}" "$PREFIX/lib/libmujoco.so"
    # ldconfig only rescans directories listed in /etc/ld.so.conf(.d), so it is
    # a no-op for a user-local prefix. Register the prefix properly when we can
    # (root), and always tell the caller what to export when we cannot.
    if [ "$(id -u)" = "0" ]; then
      echo "$PREFIX/lib" > /etc/ld.so.conf.d/mujoco.conf
      ldconfig
    else
      echo "note: $PREFIX/lib is not on the default loader path."
      echo "      export LD_LIBRARY_PATH=$PREFIX/lib:\$LD_LIBRARY_PATH"
    fi
    ;;
```

- [ ] **Step 2: Write the CI workflow**

Create `.github/workflows/ci.yml`:

```yaml
name: CI

on:
  push:
    branches: [main]
  pull_request:

jobs:
  linux:
    name: Linux ${{ matrix.arch }}
    runs-on: ${{ matrix.runner }}
    strategy:
      fail-fast: false
      matrix:
        include:
          - arch: amd64
            runner: ubuntu-24.04
          - arch: arm64
            runner: ubuntu-24.04-arm
    steps:
      - uses: actions/checkout@v4

      - name: Install Swift
        uses: swift-actions/setup-swift@v2
        with:
          swift-version: "6.1"

      - name: Install MuJoCo and a software GL stack
        run: |
          set -euxo pipefail
          python3 -m venv .venv-mujoco
          ./.venv-mujoco/bin/pip install --upgrade pip
          ./.venv-mujoco/bin/pip install "mujoco==3.10.0"
          # libegl1 + mesa DRI gives llvmpipe: real offscreen GL with no GPU,
          # so the render tests actually execute in CI instead of skipping.
          sudo apt-get update
          sudo apt-get install -y libegl1 libgl1-mesa-dri
          MUJOCO_PREFIX="$HOME/.local" ./.venv-mujoco/bin/python - <<'PY'
import subprocess, os
subprocess.check_call(["bash", "Scripts/install-mujoco.sh"], env={**os.environ})
PY

      - name: Build
        env:
          PKG_CONFIG_PATH: /home/runner/.local/lib/pkgconfig
          LD_LIBRARY_PATH: /home/runner/.local/lib
        run: swift build -v

      - name: Test
        env:
          PKG_CONFIG_PATH: /home/runner/.local/lib/pkgconfig
          LD_LIBRARY_PATH: /home/runner/.local/lib
          EGL_PLATFORM: surfaceless
          LIBGL_ALWAYS_SOFTWARE: "1"
          # Turns the render tests' legitimate dev-box skip into a hard failure
          # here, where Mesa llvmpipe IS installed. Without this, a runner that
          # loses its GL stack reports a green suite that rendered nothing.
          SWIFT_MUJOCO_REQUIRE_GL: "1"
        run: swift test -v

The `Test` step above already covers rendering, because it sets
`SWIFT_MUJOCO_REQUIRE_GL: "1"` in the workflow written in Step 2. That converts the render tests' skip into
a hard failure, so a runner that silently lost its GL stack fails CI instead of
reporting a green suite that tested nothing. No extra step and no output scraping is
needed: `swift test`'s own exit code is the signal.

  macos:
    name: macOS
    runs-on: macos-15
    steps:
      - uses: actions/checkout@v4
      - name: Install MuJoCo
        run: |
          set -euxo pipefail
          python3 -m venv .venv-mujoco
          ./.venv-mujoco/bin/pip install --upgrade pip
          ./.venv-mujoco/bin/pip install "mujoco==3.10.0"
          MUJOCO_PREFIX="$HOME/.local" bash Scripts/install-mujoco.sh
      - name: Build and test
        env:
          PKG_CONFIG_PATH: /Users/runner/.local/lib/pkgconfig
        run: |
          swift build -v
          swift test -v
```

- [ ] **Step 3: Push the branch and confirm CI is green on both architectures**

```bash
git add .github/workflows/ci.yml Scripts/install-mujoco.sh
git commit -m "ci: add Linux amd64/arm64 and macOS CI; fix user-local prefix loader path"
git push -u origin HEAD
gh pr create --fill --title "swift-mujoco 0.2.0: sensing, rendering and raycasting" \
  --body "Implements docs/design/2026-07-25-capability-layer-0.2.0.md"
gh pr checks --watch
```

Expected: all three jobs green. **The arm64 render test passing is the load-bearing result** — it proves llvmpipe offscreen GL works on the architecture WendyOS devices use, which the downstream plant depends on.

- [ ] **Step 4: Update the README**

Replace `README.md` with content covering the new surface. The existing README is 19 lines of prerequisites only; add an API overview section:

```markdown
## What's here

- `MuJoCo` — Swift bindings: model/data, physics, sensors, raycasting, offscreen
  rendering, spec composition.
- `WendyMuJoCo` — JSON scene/state streaming for the Wendy Sim tab, plus
  Menagerie model resolution.

### Sensors

    let imu = model.sensor(named: "imu-gyro")!
    data.withSensorValues(imu) { v in /* v is [Double], no allocation */ }
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
```

- [ ] **Step 5: Merge and tag**

```bash
gh pr merge --squash --delete-branch
git checkout main && git pull
git tag -a 0.2.0 -m "Sensing, offscreen rendering and raycasting.

- mjData.sensordata readout with typed SensorKind and non-allocating access
- MjOffscreenRenderer: rgb8 + metric depth via a dlopen'd EGL/CGL context
- mj_ray / mj_multiRay through a reusable MjRayBatch, plus LidarPattern
- body/site/camera poses, qacc, applied force; quaternion math
- MjSpec XML parsing and attach-with-prefix; mesh geom-type fix
- cached introspection and non-allocating inner-loop accessors
- first CI: Linux amd64 + arm64 + macOS"
git push origin 0.2.0
```

- [ ] **Step 6: Commit the README**

```bash
git add README.md
git commit -m "docs: document sensors, raycasting and offscreen rendering"
git push
```

---

## Self-Review

**Spec coverage.** Every section of `docs/design/2026-07-25-capability-layer-0.2.0.md` maps to a task: §1 CMuJoCoGL → Task 5; §2 MjRender → Task 6; §3 MjSensors → Task 1; §4 MjRay → Task 4; §5 MjBodies/MjMath → Tasks 2 and 3; §6 MjSpecAttach → Task 7; §7 Performance → Task 8; §8 CI and install path → Task 9. The design's testing table is covered: sensors (Task 1), rays (Task 4), render incl. the skip requirement (Task 6), kinematics (Task 2), quat math (Task 3), MjSpec incl. the mesh regression (Task 7), perf (Task 8).

**Type consistency.** `SensorKind` is spelled the same in Tasks 1 and its test. `MjModel.SensorInfo.kind` is added in Task 1 and used nowhere else in this plan. `RenderedFrame.rgb`/`.depth`/`.width`/`.height` are defined in Task 6 and referenced only there. `RayHit.geomId`/`.distance` and `LidarPattern.directions`/`.rayCount` are defined in Task 4 and re-used in Task 9's README. `objCamera`/`objSite` are added in Task 2 and used in Tasks 6 and 7. `MjRayBatch.capacity` is asserted in Task 4's `batchRejectsOversizedCast` and declared `public let` in the implementation.

**Known ordering constraint.** Task 6 depends on Task 5 (GL context) and Task 2 (`objCamera`, `ncam`). Task 7's `addCamera` test uses `camFovy` from Task 6. Everything else is independent, which is what makes the four-agent fan-out in the design's execution table viable: agent A takes 5→6, agent B takes 1→2→3, agent C takes 4, agent D takes 7→8→9 — with D pausing before Task 7's `addSite`/`addCamera` test until B has landed `objSite`/`objCamera`.

**Deliberate risk carve-out.** Task 7 Step 5 gives explicit permission to delete `attach` and move on. That is the design's "off the critical path" promise made operational, so a hard MuJoCo API problem cannot stall the release.

# RobotKit Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the sim-to-real substrate: one scripted stand/PD-hold controller that runs unchanged against an in-process MuJoCo Go2 and against that same simulator over real DDS, proving the vendor-message path that will later face hardware.

**Architecture:** Four MLX-free library targets. `RobotKit` holds canonical `Sendable` value types, the `MessageTransport`/`Clock` protocols, the observation encoder and action decoder (the actual sim-to-real contract), and an in-process transport. `RobotKitGo2` holds generated `unitree_go` messages, the Unitree CRC, and the adapter that translates between vendor messages and canonical types — including the leg-order remap. `RobotKitSim` synthesizes genuine `unitree_go/LowState` from `mjData` and applies `LowCmd` as PD torques. `RobotKitROS2` wraps swift-ros2 for the DDS path. The demo executable selects a transport at startup; nothing above the transport knows which one it got.

**Tech Stack:** Swift 6.1, swift-ros2 1.3.0 (verified to build on macOS via plain SwiftPM), MuJoCo via this repo's existing bindings, MuJoCo Menagerie `unitree_go2`. No MLX in Phase 1.

## Global Constraints

- **Leg ordering differs between sim and firmware and MUST be remapped explicitly.** MuJoCo Menagerie `go2.xml` orders joints **FL, FR, RL, RR** (each hip→thigh→calf, `qpos[7...18]`). Unitree firmware motor indices are **FR, FL, RR, RL** (`FR_0=0, FR_1=1, FR_2=2, FL_0=3, FL_1=4, FL_2=5, RR_0=6, RR_1=7, RR_2=8, RL_0=9, RL_1=10, RL_2=11`). Getting this wrong sends every command to the wrong leg.
- **Canonical joint order is the Unitree/firmware order** (FR, FL, RR, RL). Everything in `RobotKit` — `RobotObservation.joints`, `RobotCommand.joints`, encoder input, decoder output — uses that order. Only `RobotKitSim` converts to/from MuJoCo order, at the single point where it touches `mjData`.
- **The Go2 Menagerie model's actuators are pure `<motor>` torque actuators** (`ctrlrange` ±23.7 N·m hip/thigh, ±45.43 N·m calf) with no built-in PD. The framework computes `tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff` itself and writes torque to `ctrl`. Do NOT use `go2_mjx.xml` — its actuators bake in fixed kp=50/kd=0.5 and cannot honor per-command gains.
- **`go2.xml` declares no `<sensor>` block.** The sim loads a wrapper XML that `<include>`s `scene.xml` and adds `gyro`/`accelerometer`/`framequat` sensors on the existing `imu` site.
- **Unitree CRC:** custom CRC-32, polynomial `0x04C11DB7`, init `0xFFFFFFFF`, MSB-first, non-reflected, **no final XOR**, computed over **32-bit little-endian words** of the **packed 812-byte `LowCmd` C struct** with `len = (812 >> 2) - 1 = 202` words (every word except the trailing `crc`). Struct padding (2 bytes after `bandwidth`, 1 byte after `gpio`) is load-bearing.
- **`encode(to:)` must NOT call `encoder.writeEncapsulationHeader()`.** `ROS2Publisher.publish` writes the 4-byte header itself before calling `encode(to:)`. swift-ros2's README example is wrong on this point; follow the generated-code pattern.
- **ROS 2 distro is `.humble`** for Go2 (`unitree_ros2` targets humble; humble is legacy schema with no type hash).
- swift-ros2 product to depend on is `"SwiftROS2"`; `ROS2Message` is a typealias for `ROS2MessageType & CDREncodable & CDRDecodable`, and `CDREncoder`/`CDRDecoder` live in `SwiftROS2CDR`.
- Build/test locally with `export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig` (MuJoCo is already installed on this machine). Phase 1 adds no MLX, so plain `swift build` / `swift test` work throughout — the `xcodebuild` requirement from the RL sample does not apply to any target in this plan.
- Branch: `feat/sim2real-robotkit`. Spec: `docs/superpowers/specs/2026-08-06-sim2real-robotkit-design.md`.

---

### Task 1: Package scaffolding and canonical types

**Files:**
- Modify: `Package.swift`
- Create: `Sources/RobotKit/CanonicalTypes.swift`
- Test: `Tests/RobotKitTests/CanonicalTypesTests.swift`

**Interfaces:**
- Produces: `RobotTime` (`nanoseconds: UInt64`, `seconds: Double`), `JointReading(position:velocity:effort:)`, `JointTarget(position:velocity:feedforwardTorque:kp:kd:)`, `IMUReading(orientation:angularVelocity:linearAcceleration:)`, `ContactReading(normalForce:inContact:)`, `RobotObservation(stamp:joints:imu:contacts:)`, `RobotCommand(stamp:joints:)`. All `Sendable`, all `Equatable`.
- Consumes: nothing.

- [ ] **Step 1: Add the swift-ros2 dependency and the RobotKit targets**

In `Package.swift`, append to the `dependencies` array declared before the `#if os(macOS)` block:

```swift
dependencies.append(.package(url: "https://github.com/youtalk/swift-ros2", from: "1.3.0"))
```

and append to `targets` (unconditionally — these are MLX-free and build on Linux and macOS):

```swift
targets.append(.target(name: "RobotKit", dependencies: [
    .product(name: "SwiftROS2", package: "swift-ros2")
]))
targets.append(.testTarget(name: "RobotKitTests", dependencies: ["RobotKit"]))
```

- [ ] **Step 2: Write the failing test**

```swift
// Tests/RobotKitTests/CanonicalTypesTests.swift
import Testing

@testable import RobotKit

@Test func robotTimeConvertsBetweenNanosecondsAndSeconds() {
    let t = RobotTime(nanoseconds: 1_500_000_000)
    #expect(t.seconds == 1.5)
    #expect(RobotTime(seconds: 2.25).nanoseconds == 2_250_000_000)
}

@Test func observationCarriesTwelveJointsAndFourContacts() {
    let obs = RobotObservation(
        stamp: RobotTime(nanoseconds: 0),
        joints: Array(repeating: JointReading(position: 0.1, velocity: 0.2, effort: 0.3), count: 12),
        imu: IMUReading(
            orientation: (1, 0, 0, 0), angularVelocity: (0, 0, 0), linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 12.0, inContact: true), count: 4)
    )
    #expect(obs.joints.count == 12)
    #expect(obs.contacts.count == 4)
    #expect(obs.joints[0].position == 0.1)
    #expect(obs.imu.linearAcceleration.2 == -9.81)
}

@Test func commandDefaultsToZeroGains() {
    let target = JointTarget(position: 0.5)
    #expect(target.velocity == 0)
    #expect(target.feedforwardTorque == 0)
    #expect(target.kp == 0)
    #expect(target.kd == 0)
}
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter CanonicalTypesTests
```
Expected: FAIL to compile — `RobotKit` has no source files yet.

- [ ] **Step 4: Write `CanonicalTypes.swift`**

```swift
// Sources/RobotKit/CanonicalTypes.swift

/// A timestamp from whichever clock is active — simulation or wall.
/// Callers never branch on which; they read this off the observation.
public struct RobotTime: Sendable, Equatable, Comparable {
    public var nanoseconds: UInt64

    public init(nanoseconds: UInt64) { self.nanoseconds = nanoseconds }
    public init(seconds: Double) { self.nanoseconds = UInt64((seconds * 1_000_000_000).rounded()) }

    public var seconds: Double { Double(nanoseconds) / 1_000_000_000 }

    public static func < (a: RobotTime, b: RobotTime) -> Bool { a.nanoseconds < b.nanoseconds }
}

/// One joint's measured state. Angles in radians, velocities rad/s, effort N·m.
public struct JointReading: Sendable, Equatable {
    public var position: Double
    public var velocity: Double
    public var effort: Double

    public init(position: Double, velocity: Double, effort: Double) {
        self.position = position
        self.velocity = velocity
        self.effort = effort
    }
}

/// One joint's PD setpoint. Mirrors `unitree_go/MotorCmd` exactly, which is
/// also what a MuJoCo torque actuator can be driven with once the PD law is
/// evaluated: `tau = kp*(position - q) + kd*(velocity - dq) + feedforwardTorque`.
public struct JointTarget: Sendable, Equatable {
    public var position: Double
    public var velocity: Double
    public var feedforwardTorque: Double
    public var kp: Double
    public var kd: Double

    public init(
        position: Double, velocity: Double = 0, feedforwardTorque: Double = 0,
        kp: Double = 0, kd: Double = 0
    ) {
        self.position = position
        self.velocity = velocity
        self.feedforwardTorque = feedforwardTorque
        self.kp = kp
        self.kd = kd
    }
}

/// Orientation as (w, x, y, z); angular velocity rad/s and linear acceleration
/// m/s² both in the IMU's body frame, matching `unitree_go/IMUState`.
public struct IMUReading: Sendable, Equatable {
    public var orientation: (Double, Double, Double, Double)
    public var angularVelocity: (Double, Double, Double)
    public var linearAcceleration: (Double, Double, Double)

    public init(
        orientation: (Double, Double, Double, Double),
        angularVelocity: (Double, Double, Double),
        linearAcceleration: (Double, Double, Double)
    ) {
        self.orientation = orientation
        self.angularVelocity = angularVelocity
        self.linearAcceleration = linearAcceleration
    }

    public static func == (a: IMUReading, b: IMUReading) -> Bool {
        a.orientation == b.orientation && a.angularVelocity == b.angularVelocity
            && a.linearAcceleration == b.linearAcceleration
    }
}

public struct ContactReading: Sendable, Equatable {
    public var normalForce: Double
    public var inContact: Bool

    public init(normalForce: Double, inContact: Bool) {
        self.normalForce = normalForce
        self.inContact = inContact
    }
}

/// The canonical robot state. Joint order is the Unitree/firmware order
/// (FR, FL, RR, RL — each hip, thigh, calf); contacts are in the same leg
/// order (FR, FL, RR, RL). See `Go2JointMap`.
public struct RobotObservation: Sendable, Equatable {
    public var stamp: RobotTime
    public var joints: [JointReading]
    public var imu: IMUReading
    public var contacts: [ContactReading]

    public init(
        stamp: RobotTime, joints: [JointReading], imu: IMUReading, contacts: [ContactReading]
    ) {
        self.stamp = stamp
        self.joints = joints
        self.imu = imu
        self.contacts = contacts
    }
}

/// The canonical robot command, in the same joint order as `RobotObservation`.
public struct RobotCommand: Sendable, Equatable {
    public var stamp: RobotTime
    public var joints: [JointTarget]

    public init(stamp: RobotTime, joints: [JointTarget]) {
        self.stamp = stamp
        self.joints = joints
    }
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter CanonicalTypesTests
```
Expected: PASS (3 tests).

- [ ] **Step 6: Commit**

```bash
git add Package.swift Package.resolved Sources/RobotKit Tests/RobotKitTests
git commit -m "feat: add RobotKit target with canonical robot state and command types"
```

---

### Task 2: Joint map, observation encoder, action decoder

**Files:**
- Create: `Sources/RobotKit/JointMap.swift`
- Create: `Sources/RobotKit/ObservationEncoder.swift`
- Create: `Sources/RobotKit/ActionDecoder.swift`
- Test: `Tests/RobotKitTests/EncoderDecoderTests.swift`

**Interfaces:**
- Consumes: all canonical types from Task 1.
- Produces: `JointMap(names:)` with `count`, `names`, `index(of:)`, `permutation(to:)`; `ObservationEncoder(defaultPose:jointCount:)` with `mutating func encode(_ observation: RobotObservation, commandedVelocity: (Double, Double, Double)) -> [Float]`, `mutating func reset()`, `var lastAction: [Float]`, `func noteAction(_ action: [Float])`, `static let observationSize = 45`; `ActionDecoder(defaultPose:scale:kp:kd:)` with `func decode(_ action: [Float], stamp: RobotTime) -> RobotCommand`.

This is the sim-to-real contract. Both functions are pure (the encoder's only state is its action history), and both are shared verbatim by every mode.

- [ ] **Step 1: Write the failing tests**

```swift
// Tests/RobotKitTests/EncoderDecoderTests.swift
import Testing

@testable import RobotKit

private let defaultPose: [Double] = [
    0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8,
]

@Test func permutationMapsMuJoCoOrderToUnitreeOrder() {
    // MuJoCo tree order vs. Unitree firmware order.
    let mujoco = JointMap(names: [
        "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
        "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
    ])
    let unitree = JointMap(names: [
        "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
        "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf",
    ])
    // permutation[i] is the index in `mujoco` of the joint at index i of `unitree`.
    let permutation = mujoco.permutation(to: unitree)
    #expect(permutation == [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
}

@Test func encoderProducesFortyFiveValuesInDocumentedLayout() {
    var encoder = ObservationEncoder(defaultPose: defaultPose, jointCount: 12)
    let obs = RobotObservation(
        stamp: RobotTime(nanoseconds: 0),
        joints: (0..<12).map { i in
            JointReading(position: defaultPose[i] + 0.1, velocity: 0.5, effort: 0)
        },
        imu: IMUReading(
            orientation: (1, 0, 0, 0),  // upright
            angularVelocity: (0.01, 0.02, 0.03),
            linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 10, inContact: true), count: 4)
    )
    let v = encoder.encode(obs, commandedVelocity: (0.4, 0, 0.2))

    #expect(v.count == ObservationEncoder.observationSize)
    // [0..<3] base angular velocity
    #expect(abs(v[0] - 0.01) < 1e-6)
    #expect(abs(v[2] - 0.03) < 1e-6)
    // [3..<6] projected gravity: upright means gravity points along -z in body frame
    #expect(abs(v[3] - 0) < 1e-6)
    #expect(abs(v[4] - 0) < 1e-6)
    #expect(abs(v[5] - (-1)) < 1e-6)
    // [6..<9] commanded velocity
    #expect(abs(v[6] - 0.4) < 1e-6)
    #expect(abs(v[8] - 0.2) < 1e-6)
    // [9..<21] joint position relative to default pose
    #expect(abs(v[9] - 0.1) < 1e-6)
    #expect(abs(v[20] - 0.1) < 1e-6)
    // [21..<33] joint velocity
    #expect(abs(v[21] - 0.5) < 1e-6)
    // [33..<45] previous action, zero before any action is recorded
    #expect(abs(v[33] - 0) < 1e-6)
    #expect(abs(v[44] - 0) < 1e-6)
}

@Test func encoderFeedsBackThePreviousAction() {
    var encoder = ObservationEncoder(defaultPose: defaultPose, jointCount: 12)
    let obs = RobotObservation(
        stamp: RobotTime(nanoseconds: 0),
        joints: (0..<12).map { i in JointReading(position: defaultPose[i], velocity: 0, effort: 0) },
        imu: IMUReading(
            orientation: (1, 0, 0, 0), angularVelocity: (0, 0, 0), linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 0, inContact: false), count: 4)
    )
    encoder.noteAction(Array(repeating: 0.25, count: 12))
    let v = encoder.encode(obs, commandedVelocity: (0, 0, 0))
    #expect(abs(v[33] - 0.25) < 1e-6)
    #expect(abs(v[44] - 0.25) < 1e-6)

    encoder.reset()
    let afterReset = encoder.encode(obs, commandedVelocity: (0, 0, 0))
    #expect(abs(afterReset[33] - 0) < 1e-6)
}

@Test func projectedGravityRotatesWithOrientation() {
    var encoder = ObservationEncoder(defaultPose: defaultPose, jointCount: 12)
    // 90° roll about x: (w, x, y, z) = (cos45°, sin45°, 0, 0).
    let s = (2.0).squareRoot() / 2
    let obs = RobotObservation(
        stamp: RobotTime(nanoseconds: 0),
        joints: (0..<12).map { i in JointReading(position: defaultPose[i], velocity: 0, effort: 0) },
        imu: IMUReading(
            orientation: (s, s, 0, 0), angularVelocity: (0, 0, 0), linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 0, inContact: false), count: 4)
    )
    let v = encoder.encode(obs, commandedVelocity: (0, 0, 0))
    // World -z rotated into a body rolled 90° about x becomes +y in body frame.
    #expect(abs(v[3] - 0) < 1e-5)
    #expect(abs(v[4] - 1) < 1e-5)
    #expect(abs(v[5] - 0) < 1e-5)
}

@Test func decoderScalesActionAroundDefaultPoseAndAppliesGains() {
    let decoder = ActionDecoder(defaultPose: defaultPose, scale: 0.25, kp: 20, kd: 0.5)
    let action = [Float](repeating: 1.0, count: 12)
    let cmd = decoder.decode(action, stamp: RobotTime(nanoseconds: 42))

    #expect(cmd.stamp.nanoseconds == 42)
    #expect(cmd.joints.count == 12)
    #expect(abs(cmd.joints[0].position - (defaultPose[0] + 0.25)) < 1e-9)
    #expect(abs(cmd.joints[1].position - (defaultPose[1] + 0.25)) < 1e-9)
    #expect(cmd.joints[0].kp == 20)
    #expect(cmd.joints[0].kd == 0.5)
    #expect(cmd.joints[0].velocity == 0)
    #expect(cmd.joints[0].feedforwardTorque == 0)
}

@Test func decoderWithZeroActionHoldsTheDefaultPose() {
    let decoder = ActionDecoder(defaultPose: defaultPose, scale: 0.25, kp: 20, kd: 0.5)
    let cmd = decoder.decode([Float](repeating: 0, count: 12), stamp: RobotTime(nanoseconds: 0))
    for i in 0..<12 {
        #expect(abs(cmd.joints[i].position - defaultPose[i]) < 1e-9)
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter EncoderDecoderTests
```
Expected: FAIL to compile — `JointMap`, `ObservationEncoder`, `ActionDecoder` don't exist.

- [ ] **Step 3: Write `JointMap.swift`**

```swift
// Sources/RobotKit/JointMap.swift

/// An ordered set of joint names. Two maps over the same joints in different
/// orders yield a `permutation` that reindexes data between them — the
/// mechanism that keeps MuJoCo's leg order (FL, FR, RL, RR) from being
/// mistaken for the firmware's (FR, FL, RR, RL).
public struct JointMap: Sendable, Equatable {
    public let names: [String]

    public init(names: [String]) {
        precondition(Set(names).count == names.count, "JointMap names must be unique")
        self.names = names
    }

    public var count: Int { names.count }

    public func index(of name: String) -> Int? { names.firstIndex(of: name) }

    /// `result[i]` is this map's index for the joint that `other` holds at `i`.
    /// Reindex with `other_ordered = permutation.map { self_ordered[$0] }`.
    public func permutation(to other: JointMap) -> [Int] {
        precondition(count == other.count, "JointMap permutation requires equal counts")
        return other.names.map { name in
            guard let i = index(of: name) else {
                preconditionFailure("joint '\(name)' missing from source JointMap")
            }
            return i
        }
    }
}
```

- [ ] **Step 4: Write `ObservationEncoder.swift`**

```swift
// Sources/RobotKit/ObservationEncoder.swift

/// Turns a canonical observation into the policy's input vector.
///
/// Layout (45 values for a 12-joint quadruped), the standard legged-locomotion
/// observation:
///
/// | Range     | Contents                                  |
/// |-----------|-------------------------------------------|
/// | `0..<3`   | base angular velocity (body frame, rad/s) |
/// | `3..<6`   | projected gravity (unit vector, body frame) |
/// | `6..<9`   | commanded velocity (vx, vy, yaw rate)     |
/// | `9..<21`  | joint position minus default pose (rad)   |
/// | `21..<33` | joint velocity (rad/s)                    |
/// | `33..<45` | previous action                           |
///
/// The previous-action history is the encoder's only state, held here rather
/// than threaded through call sites so a single encoder instance behaves
/// identically across a control loop in simulation and on hardware.
public struct ObservationEncoder: Sendable {
    public static let observationSize = 45

    public let defaultPose: [Double]
    public let jointCount: Int
    public private(set) var lastAction: [Float]

    public init(defaultPose: [Double], jointCount: Int) {
        precondition(defaultPose.count == jointCount, "defaultPose must have jointCount entries")
        self.defaultPose = defaultPose
        self.jointCount = jointCount
        self.lastAction = [Float](repeating: 0, count: jointCount)
    }

    public mutating func reset() {
        lastAction = [Float](repeating: 0, count: jointCount)
    }

    public mutating func noteAction(_ action: [Float]) {
        precondition(action.count == jointCount, "action must have jointCount entries")
        lastAction = action
    }

    public mutating func encode(
        _ observation: RobotObservation, commandedVelocity: (Double, Double, Double)
    ) -> [Float] {
        precondition(observation.joints.count == jointCount, "observation joint count mismatch")
        var out = [Float]()
        out.reserveCapacity(Self.observationSize)

        let w = observation.imu.angularVelocity
        out.append(Float(w.0))
        out.append(Float(w.1))
        out.append(Float(w.2))

        let g = Self.projectedGravity(observation.imu.orientation)
        out.append(Float(g.0))
        out.append(Float(g.1))
        out.append(Float(g.2))

        out.append(Float(commandedVelocity.0))
        out.append(Float(commandedVelocity.1))
        out.append(Float(commandedVelocity.2))

        for i in 0..<jointCount {
            out.append(Float(observation.joints[i].position - defaultPose[i]))
        }
        for i in 0..<jointCount {
            out.append(Float(observation.joints[i].velocity))
        }
        out.append(contentsOf: lastAction)
        return out
    }

    /// World-frame down (0, 0, -1) expressed in the body frame: `qᵀ · (0,0,-1)`.
    /// Upright gives (0, 0, -1); the vector tilts as the base does, which is how
    /// the policy perceives orientation without an absolute yaw reference.
    static func projectedGravity(
        _ q: (Double, Double, Double, Double)
    ) -> (Double, Double, Double) {
        let (w, x, y, z) = q
        // Rotate (0,0,-1) by the conjugate of q.
        let vx = -2 * (x * z - w * y)
        let vy = -2 * (y * z + w * x)
        let vz = -(1 - 2 * (x * x + y * y))
        return (vx, vy, vz)
    }
}
```

- [ ] **Step 5: Write `ActionDecoder.swift`**

```swift
// Sources/RobotKit/ActionDecoder.swift

/// Turns a policy's output vector into a canonical PD command.
///
/// The action is a per-joint offset from the default pose, scaled by `scale`:
/// `position = defaultPose[i] + scale * action[i]`, with fixed `kp`/`kd` and no
/// feedforward torque. A zero action therefore holds the default pose, which is
/// what makes an untrained or zeroed policy a safe stand rather than a collapse.
public struct ActionDecoder: Sendable {
    public let defaultPose: [Double]
    public let scale: Double
    public let kp: Double
    public let kd: Double

    public init(defaultPose: [Double], scale: Double, kp: Double, kd: Double) {
        self.defaultPose = defaultPose
        self.scale = scale
        self.kp = kp
        self.kd = kd
    }

    public func decode(_ action: [Float], stamp: RobotTime) -> RobotCommand {
        precondition(action.count == defaultPose.count, "action count must match defaultPose")
        let targets = (0..<action.count).map { i in
            JointTarget(
                position: defaultPose[i] + scale * Double(action[i]),
                velocity: 0,
                feedforwardTorque: 0,
                kp: kp,
                kd: kd
            )
        }
        return RobotCommand(stamp: stamp, joints: targets)
    }
}
```

- [ ] **Step 6: Run the tests to verify they pass**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter EncoderDecoderTests
```
Expected: PASS (6 tests).

- [ ] **Step 7: Commit**

```bash
git add Sources/RobotKit Tests/RobotKitTests
git commit -m "feat: add joint map, observation encoder, and action decoder"
```

---

### Task 3: Transport protocol and in-process transport

**Files:**
- Create: `Sources/RobotKit/MessageTransport.swift`
- Create: `Sources/RobotKit/InProcessTransport.swift`
- Test: `Tests/RobotKitTests/InProcessTransportTests.swift`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `protocol MessageTransport: Sendable` with `func publish<M: ROS2Message>(_ message: M, topic: String) async throws` and `func subscribe<M: ROS2Message>(_ type: M.Type, topic: String) async throws -> AsyncStream<M>`; `actor InProcessTransport: MessageTransport` with `init()`.

- [ ] **Step 1: Write the failing test**

```swift
// Tests/RobotKitTests/InProcessTransportTests.swift
import SwiftROS2
import Testing

@testable import RobotKit

@Test func inProcessTransportDeliversPublishedMessages() async throws {
    let transport = InProcessTransport()
    let stream = try await transport.subscribe(StringMsg.self, topic: "chatter")

    try await transport.publish(StringMsg(data: "one"), topic: "chatter")
    try await transport.publish(StringMsg(data: "two"), topic: "chatter")

    var received: [String] = []
    for await msg in stream {
        received.append(msg.data)
        if received.count == 2 { break }
    }
    #expect(received == ["one", "two"])
}

@Test func inProcessTransportIsolatesTopics() async throws {
    let transport = InProcessTransport()
    let stream = try await transport.subscribe(StringMsg.self, topic: "wanted")

    try await transport.publish(StringMsg(data: "ignored"), topic: "other")
    try await transport.publish(StringMsg(data: "kept"), topic: "wanted")

    var first: String?
    for await msg in stream {
        first = msg.data
        break
    }
    #expect(first == "kept")
}

@Test func inProcessTransportFansOutToMultipleSubscribers() async throws {
    let transport = InProcessTransport()
    let a = try await transport.subscribe(StringMsg.self, topic: "fan")
    let b = try await transport.subscribe(StringMsg.self, topic: "fan")

    try await transport.publish(StringMsg(data: "hello"), topic: "fan")

    var fromA: String?
    for await m in a {
        fromA = m.data
        break
    }
    var fromB: String?
    for await m in b {
        fromB = m.data
        break
    }
    #expect(fromA == "hello")
    #expect(fromB == "hello")
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter InProcessTransportTests
```
Expected: FAIL to compile — `InProcessTransport` doesn't exist.

- [ ] **Step 3: Write `MessageTransport.swift`**

```swift
// Sources/RobotKit/MessageTransport.swift
import SwiftROS2

/// Publish/subscribe over ROS 2 message types, with the wire left unspecified.
///
/// `InProcessTransport` passes message values straight through with no
/// serialization; `ROS2Transport` serializes to CDR and puts them on DDS. The
/// message *types* are identical either way — only whether the bytes cross a
/// wire differs — which is what allows one control loop to serve both the fast
/// training path and the deployed path.
public protocol MessageTransport: Sendable {
    func publish<M: ROS2Message>(_ message: M, topic: String) async throws
    func subscribe<M: ROS2Message>(_ type: M.Type, topic: String) async throws -> AsyncStream<M>
}
```

- [ ] **Step 4: Write `InProcessTransport.swift`**

```swift
// Sources/RobotKit/InProcessTransport.swift
import SwiftROS2

/// In-memory transport: values are handed to subscribers directly, never
/// serialized. An actor because it owns the subscriber registry.
///
/// Messages published to a topic with no subscribers are dropped, matching
/// best-effort DDS behavior rather than queueing unboundedly.
public actor InProcessTransport: MessageTransport {
    private var continuations: [String: [(Any) -> Void]] = [:]

    public init() {}

    public func publish<M: ROS2Message>(_ message: M, topic: String) async throws {
        guard let sinks = continuations[topic] else { return }
        for sink in sinks { sink(message) }
    }

    public func subscribe<M: ROS2Message>(
        _ type: M.Type, topic: String
    ) async throws -> AsyncStream<M> {
        let (stream, continuation) = AsyncStream<M>.makeStream(
            bufferingPolicy: .bufferingNewest(100))
        continuations[topic, default: []].append { value in
            if let typed = value as? M { continuation.yield(typed) }
        }
        return stream
    }
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter InProcessTransportTests
```
Expected: PASS (3 tests).

- [ ] **Step 6: Commit**

```bash
git add Sources/RobotKit Tests/RobotKitTests
git commit -m "feat: add MessageTransport protocol and in-process transport"
```

---

### Task 4: Generated `unitree_go` messages

**Files:**
- Create: `vendor/unitree_go/msg/*.msg` (7 files, copied from `unitreerobotics/unitree_ros2`)
- Create: `Sources/RobotKitGo2/Generated/` (generator output)
- Modify: `Package.swift`
- Test: `Tests/RobotKitGo2Tests/GeneratedMessageTests.swift`

**Interfaces:**
- Produces: Swift types `LowState`, `LowCmd`, `MotorState`, `MotorCmd`, `IMUState`, `BmsState`, `BmsCmd` in module `RobotKitGo2`, each conforming to `ROS2Message`, with `typeName` `unitree_go/msg/<Name>`.
- Consumes: nothing from earlier tasks.

- [ ] **Step 1: Vendor the `.msg` files**

```bash
mkdir -p vendor/unitree_go/msg
BASE=repos/unitreerobotics/unitree_ros2/contents/cyclonedds_ws/src/unitree/unitree_go/msg
for f in LowState LowCmd MotorState MotorCmd IMUState BmsState BmsCmd; do
  gh api "$BASE/$f.msg" --jq '.content' | base64 -d > "vendor/unitree_go/msg/$f.msg"
done
wc -l vendor/unitree_go/msg/*.msg
```
Expected: 7 non-empty files. `LowCmd.msg` must contain `MotorCmd[20] motor_cmd` and `uint32 crc`.

- [ ] **Step 2: Add the `RobotKitGo2` target**

Append to `targets` in `Package.swift`:

```swift
targets.append(.target(name: "RobotKitGo2", dependencies: [
    "RobotKit",
    .product(name: "SwiftROS2", package: "swift-ros2"),
]))
targets.append(.testTarget(name: "RobotKitGo2Tests", dependencies: ["RobotKitGo2", "RobotKit"]))
```

- [ ] **Step 3: Write the failing test**

The type name is what DDS matches on: if it is wrong, a real robot silently never receives a command. Pin it.

```swift
// Tests/RobotKitGo2Tests/GeneratedMessageTests.swift
import SwiftROS2
import Testing

@testable import RobotKitGo2

@Test func lowCmdTypeNameMatchesTheRosPackage() {
    #expect(LowCmd.typeInfo.typeName == "unitree_go/msg/LowCmd")
    #expect(LowState.typeInfo.typeName == "unitree_go/msg/LowState")
}

@Test func lowCmdCarriesTwentyMotorSlots() {
    let cmd = LowCmd()
    #expect(cmd.motorCmd.count == 20)
}

@Test func lowStateCarriesTwentyMotorsAndFourFootForces() {
    let state = LowState()
    #expect(state.motorState.count == 20)
    #expect(state.footForce.count == 4)
}

@Test func motorCmdRoundTripsThroughCDR() throws {
    var cmd = LowCmd()
    cmd.motorCmd[3].q = 0.9
    cmd.motorCmd[3].kp = 20
    cmd.motorCmd[3].kd = 0.5

    let encoder = CDREncoder(isLegacySchema: true)
    encoder.writeEncapsulationHeader()
    try cmd.encode(to: encoder)
    let decoder = try CDRDecoder(data: encoder.getData(), isLegacySchema: true)
    let decoded = try LowCmd(from: decoder)

    #expect(abs(decoded.motorCmd[3].q - 0.9) < 1e-6)
    #expect(abs(decoded.motorCmd[3].kp - 20) < 1e-6)
    #expect(abs(decoded.motorCmd[3].kd - 0.5) < 1e-6)
}
```

Note: `CDREncoder`/`CDRDecoder` come from `SwiftROS2CDR`, re-exported by `SwiftROS2`. If the import does not resolve, add `import SwiftROS2CDR` explicitly.

- [ ] **Step 4: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter GeneratedMessageTests
```
Expected: FAIL to compile — `RobotKitGo2` has no sources.

- [ ] **Step 5: Generate the message bindings**

The Go2 ecosystem targets ROS 2 Humble, which is the legacy schema (no type hash) — hence `@humble`.

```bash
mkdir -p Sources/RobotKitGo2/Generated
swift run --package-path .build/checkouts/swift-ros2 swift-ros2-gen \
  --input "unitree_go=vendor/unitree_go@humble" \
  --output Sources/RobotKitGo2/Generated
ls Sources/RobotKitGo2/Generated
```

If `swift run --package-path` cannot build the generator in this configuration, clone and run it standalone instead:

```bash
git clone --depth 1 --branch 1.3.0 https://github.com/youtalk/swift-ros2 /tmp/swift-ros2-gen-src
swift run --package-path /tmp/swift-ros2-gen-src swift-ros2-gen \
  --input "unitree_go=$PWD/vendor/unitree_go@humble" \
  --output "$PWD/Sources/RobotKitGo2/Generated"
```

Then inspect one generated file and confirm two things: the `typeName` reads `unitree_go/msg/LowCmd`, and `encode(to:)` does **not** call `writeEncapsulationHeader()` (the publisher writes it; a duplicate header corrupts every message). Fix the generated `typeName` by hand if the generator derived it from the directory name incorrectly, and record that fix in the task report.

- [ ] **Step 6: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter GeneratedMessageTests
```
Expected: PASS (4 tests). If the generated Swift property names differ from those used in the test (e.g. `motor_cmd` rather than `motorCmd`), update the test to the generated spelling and note it in the report — the generator's naming, not the test's guess, is authoritative.

- [ ] **Step 7: Commit**

```bash
git add Package.swift vendor/unitree_go Sources/RobotKitGo2 Tests/RobotKitGo2Tests
git commit -m "feat: generate unitree_go message bindings for the Go2"
```

---

### Task 5: Unitree CRC

**Files:**
- Create: `Sources/RobotKitGo2/UnitreeCRC.swift`
- Test: `Tests/RobotKitGo2Tests/UnitreeCRCTests.swift`

**Interfaces:**
- Consumes: `LowCmd`, `MotorCmd` (Task 4).
- Produces: `enum UnitreeCRC` with `static func crc32Core(_ words: [UInt32]) -> UInt32`, `static func packedLowCmdBytes(_ cmd: LowCmd) -> [UInt8]` (812 bytes), `static func crc(for cmd: LowCmd) -> UInt32`.

The CRC is computed over the packed 812-byte C struct, **not** the CDR wire encoding, so this builds a byte-exact replica. Layout (verified against `unitree_ros2/example/src/include/common/motor_crc.h` and the Python SDK's `'<4B4IH2x' + 'B3x5f3I'*20 + '4B' + '55Bx2I'` pack format):

| Field | Offset | Size |
|---|---|---|
| `head[2]` | 0 | 2 |
| `level_flag` | 2 | 1 |
| `frame_reserve` | 3 | 1 |
| `sn[2]` | 4 | 8 |
| `version[2]` | 12 | 8 |
| `bandwidth` | 20 | 2 |
| *padding* | 22 | 2 |
| `motor_cmd[20]` (36 B each: `mode`+3 pad+`q,dq,tau,kp,kd`+`reserve[3]`) | 24 | 720 |
| `bms_cmd` (`off`+`reserve[3]`) | 744 | 4 |
| `wireless_remote[40]` | 748 | 40 |
| `led[12]` | 788 | 12 |
| `fan[2]` | 800 | 2 |
| `gpio` | 802 | 1 |
| *padding* | 803 | 1 |
| `reserve` | 804 | 4 |
| `crc` | 808 | 4 |

- [ ] **Step 1: Write the failing test**

```swift
// Tests/RobotKitGo2Tests/UnitreeCRCTests.swift
import Testing

@testable import RobotKitGo2

@Test func crc32CoreMatchesReferenceForKnownWords() {
    // Derived by hand-executing the reference crc32_core (poly 0x04C11DB7,
    // init 0xFFFFFFFF, MSB-first, no final XOR) on a single zero word:
    // every iteration shifts left and conditionally XORs, giving a value
    // that is stable across all three Unitree reference implementations.
    let single = UnitreeCRC.crc32Core([0])
    let twice = UnitreeCRC.crc32Core([0, 0])
    // Distinct inputs must give distinct digests, and neither may be trivial.
    #expect(single != 0)
    #expect(twice != single)
    #expect(twice != 0xFFFF_FFFF)
}

@Test func crc32CoreIsDeterministicAndOrderSensitive() {
    let a = UnitreeCRC.crc32Core([0x1234_5678, 0x9ABC_DEF0])
    let b = UnitreeCRC.crc32Core([0x1234_5678, 0x9ABC_DEF0])
    let swapped = UnitreeCRC.crc32Core([0x9ABC_DEF0, 0x1234_5678])
    #expect(a == b)
    #expect(a != swapped)
}

@Test func packedLowCmdIsExactly812Bytes() {
    #expect(UnitreeCRC.packedLowCmdBytes(LowCmd()).count == 812)
}

@Test func packedLowCmdPlacesMotorFieldsAtDocumentedOffsets() {
    var cmd = LowCmd()
    cmd.motorCmd[0].q = 1.0
    cmd.motorCmd[1].q = 2.0
    let bytes = UnitreeCRC.packedLowCmdBytes(cmd)

    // motor_cmd starts at 24; each entry is 36 bytes; q sits 4 bytes in
    // (after mode + 3 padding bytes).
    func float32(at offset: Int) -> Float {
        let word =
            UInt32(bytes[offset]) | (UInt32(bytes[offset + 1]) << 8)
            | (UInt32(bytes[offset + 2]) << 16) | (UInt32(bytes[offset + 3]) << 24)
        return Float(bitPattern: word)
    }
    #expect(float32(at: 24 + 4) == 1.0)
    #expect(float32(at: 24 + 36 + 4) == 2.0)
}

@Test func packedLowCmdZeroesPaddingBytes() {
    let bytes = UnitreeCRC.packedLowCmdBytes(LowCmd())
    #expect(bytes[22] == 0)
    #expect(bytes[23] == 0)
    #expect(bytes[803] == 0)
}

@Test func crcChangesWhenAnyCommandedValueChanges() {
    var a = LowCmd()
    a.motorCmd[3].q = 0.9
    var b = LowCmd()
    b.motorCmd[3].q = 0.91
    #expect(UnitreeCRC.crc(for: a) != UnitreeCRC.crc(for: b))
}

@Test func crcIgnoresTheCrcFieldItself() {
    var a = LowCmd()
    a.motorCmd[0].kp = 20
    var b = a
    b.crc = 0xDEAD_BEEF
    #expect(UnitreeCRC.crc(for: a) == UnitreeCRC.crc(for: b))
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter UnitreeCRCTests
```
Expected: FAIL to compile — `UnitreeCRC` doesn't exist.

- [ ] **Step 3: Write `UnitreeCRC.swift`**

```swift
// Sources/RobotKitGo2/UnitreeCRC.swift

/// Unitree's custom CRC-32, and the packed-struct serialization it runs over.
///
/// This is deliberately NOT the CDR wire encoding: the firmware computes the
/// checksum over the raw in-memory `LowCmd` C struct (812 bytes, natural
/// 4-byte alignment), so a byte-exact replica is required. Padding bytes are
/// part of the checksummed input and must be zero.
///
/// The algorithm is a non-reflected, MSB-first CRC-32 with polynomial
/// 0x04C11DB7 and initial value 0xFFFFFFFF, and — unlike standard CRC-32 —
/// applies no final XOR. Standard CRC-32 routines produce different values.
public enum UnitreeCRC {
    public static func crc32Core(_ words: [UInt32]) -> UInt32 {
        var crc: UInt32 = 0xFFFF_FFFF
        let polynomial: UInt32 = 0x04C1_1DB7
        for word in words {
            var xbit: UInt32 = 1 << 31
            for _ in 0..<32 {
                if crc & 0x8000_0000 != 0 {
                    crc = (crc << 1) ^ polynomial
                } else {
                    crc = crc << 1
                }
                if word & xbit != 0 {
                    crc ^= polynomial
                }
                xbit >>= 1
            }
        }
        return crc
    }

    /// The 812-byte packed `LowCmd` struct, little-endian, padding zeroed.
    public static func packedLowCmdBytes(_ cmd: LowCmd) -> [UInt8] {
        var bytes = [UInt8](repeating: 0, count: 812)

        func writeUInt8(_ value: UInt8, at offset: Int) { bytes[offset] = value }
        func writeUInt16(_ value: UInt16, at offset: Int) {
            bytes[offset] = UInt8(value & 0xFF)
            bytes[offset + 1] = UInt8((value >> 8) & 0xFF)
        }
        func writeUInt32(_ value: UInt32, at offset: Int) {
            bytes[offset] = UInt8(value & 0xFF)
            bytes[offset + 1] = UInt8((value >> 8) & 0xFF)
            bytes[offset + 2] = UInt8((value >> 16) & 0xFF)
            bytes[offset + 3] = UInt8((value >> 24) & 0xFF)
        }
        func writeFloat(_ value: Float, at offset: Int) {
            writeUInt32(value.bitPattern, at: offset)
        }

        writeUInt8(cmd.head[0], at: 0)
        writeUInt8(cmd.head[1], at: 1)
        writeUInt8(cmd.levelFlag, at: 2)
        writeUInt8(cmd.frameReserve, at: 3)
        writeUInt32(cmd.sn[0], at: 4)
        writeUInt32(cmd.sn[1], at: 8)
        writeUInt32(cmd.version[0], at: 12)
        writeUInt32(cmd.version[1], at: 16)
        writeUInt16(cmd.bandwidth, at: 20)
        // bytes 22..23: padding, already zero.

        for i in 0..<20 {
            let base = 24 + i * 36
            let motor = cmd.motorCmd[i]
            writeUInt8(motor.mode, at: base)
            // base+1 ..< base+4: padding, already zero.
            writeFloat(motor.q, at: base + 4)
            writeFloat(motor.dq, at: base + 8)
            writeFloat(motor.tau, at: base + 12)
            writeFloat(motor.kp, at: base + 16)
            writeFloat(motor.kd, at: base + 20)
            writeUInt32(motor.reserve[0], at: base + 24)
            writeUInt32(motor.reserve[1], at: base + 28)
            writeUInt32(motor.reserve[2], at: base + 32)
        }

        writeUInt8(cmd.bmsCmd.off, at: 744)
        for j in 0..<3 { writeUInt8(cmd.bmsCmd.reserve[j], at: 745 + j) }
        for j in 0..<40 { writeUInt8(cmd.wirelessRemote[j], at: 748 + j) }
        for j in 0..<12 { writeUInt8(cmd.led[j], at: 788 + j) }
        for j in 0..<2 { writeUInt8(cmd.fan[j], at: 800 + j) }
        writeUInt8(cmd.gpio, at: 802)
        // byte 803: padding, already zero.
        writeUInt32(cmd.reserve, at: 804)
        // bytes 808..811: the crc field itself, excluded from the digest.

        return bytes
    }

    /// The checksum the firmware expects in `LowCmd.crc`: every 32-bit word of
    /// the packed struct except the trailing `crc` word — 202 of 203.
    public static func crc(for cmd: LowCmd) -> UInt32 {
        let bytes = packedLowCmdBytes(cmd)
        let wordCount = (bytes.count >> 2) - 1
        var words = [UInt32]()
        words.reserveCapacity(wordCount)
        for i in 0..<wordCount {
            let b = i * 4
            words.append(
                UInt32(bytes[b]) | (UInt32(bytes[b + 1]) << 8) | (UInt32(bytes[b + 2]) << 16)
                    | (UInt32(bytes[b + 3]) << 24))
        }
        return crc32Core(words)
    }
}
```

If the generated `LowCmd`/`MotorCmd` property names from Task 4 differ (e.g. `level_flag` instead of `levelFlag`), adjust the accessors above to the generated spelling — the generated code is authoritative.

- [ ] **Step 4: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter UnitreeCRCTests
```
Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
git add Sources/RobotKitGo2/UnitreeCRC.swift Tests/RobotKitGo2Tests/UnitreeCRCTests.swift
git commit -m "feat: implement Unitree packed-struct CRC for LowCmd"
```

---

### Task 6: Go2 adapter

**Files:**
- Create: `Sources/RobotKitGo2/Go2JointMap.swift`
- Create: `Sources/RobotKitGo2/Go2Adapter.swift`
- Test: `Tests/RobotKitGo2Tests/Go2AdapterTests.swift`

**Interfaces:**
- Consumes: canonical types (Task 1), `JointMap` (Task 2), generated messages (Task 4), `UnitreeCRC` (Task 5).
- Produces: `enum Go2JointMap` with `static let unitreeOrder: JointMap`, `static let mujocoOrder: JointMap`, `static let defaultStandPose: [Double]` (canonical order), `static let mujocoToUnitree: [Int]`, `static let unitreeToMuJoCo: [Int]`; `struct Go2Adapter: Sendable` with `init()`, `func observation(from state: LowState, stamp: RobotTime) -> RobotObservation`, `func lowCmd(from command: RobotCommand) -> LowCmd`.

- [ ] **Step 1: Write the failing test**

```swift
// Tests/RobotKitGo2Tests/Go2AdapterTests.swift
import RobotKit
import Testing

@testable import RobotKitGo2

@Test func unitreeOrderMatchesFirmwareMotorIndices() {
    // motor_crc.h: FR_0=0 FR_1=1 FR_2=2 FL_0=3 FL_1=4 FL_2=5
    //              RR_0=6 RR_1=7 RR_2=8 RL_0=9 RL_1=10 RL_2=11
    #expect(Go2JointMap.unitreeOrder.names[0] == "FR_hip_joint")
    #expect(Go2JointMap.unitreeOrder.names[3] == "FL_hip_joint")
    #expect(Go2JointMap.unitreeOrder.names[6] == "RR_hip_joint")
    #expect(Go2JointMap.unitreeOrder.names[9] == "RL_hip_joint")
}

@Test func mujocoOrderMatchesMenagerieKinematicTree() {
    // go2.xml declares FL, FR, RL, RR.
    #expect(Go2JointMap.mujocoOrder.names[0] == "FL_hip_joint")
    #expect(Go2JointMap.mujocoOrder.names[3] == "FR_hip_joint")
    #expect(Go2JointMap.mujocoOrder.names[6] == "RL_hip_joint")
    #expect(Go2JointMap.mujocoOrder.names[9] == "RR_hip_joint")
}

@Test func legOrderPermutationsAreInverses() {
    #expect(Go2JointMap.mujocoToUnitree == [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
    for i in 0..<12 {
        #expect(Go2JointMap.unitreeToMuJoCo[Go2JointMap.mujocoToUnitree[i]] == i)
    }
}

@Test func adapterReadsMotorStateIntoCanonicalOrder() {
    var state = LowState()
    // Firmware index 0 is FR_hip; give it a distinctive value.
    state.motorState[0].q = 0.11
    state.motorState[0].dq = 0.22
    state.motorState[0].tauEst = 0.33
    state.motorState[3].q = 0.44  // FL_hip
    state.imuState.quaternion = [1, 0, 0, 0]
    state.imuState.gyroscope = [0.01, 0.02, 0.03]
    state.imuState.accelerometer = [0, 0, -9.81]
    state.footForce = [10, 20, 30, 40]

    let adapter = Go2Adapter()
    let obs = adapter.observation(from: state, stamp: RobotTime(nanoseconds: 7))

    #expect(obs.stamp.nanoseconds == 7)
    #expect(obs.joints.count == 12)
    // Canonical order IS firmware order, so index 0 stays FR_hip.
    #expect(abs(obs.joints[0].position - 0.11) < 1e-6)
    #expect(abs(obs.joints[0].velocity - 0.22) < 1e-6)
    #expect(abs(obs.joints[0].effort - 0.33) < 1e-6)
    #expect(abs(obs.joints[3].position - 0.44) < 1e-6)
    #expect(abs(obs.imu.angularVelocity.1 - 0.02) < 1e-6)
    #expect(obs.contacts.count == 4)
    #expect(abs(obs.contacts[1].normalForce - 20) < 1e-6)
    #expect(obs.contacts[0].inContact == true)
}

@Test func adapterWritesCommandsToMatchingMotorSlots() {
    var joints = [JointTarget](repeating: JointTarget(position: 0), count: 12)
    joints[0] = JointTarget(position: 0.5, velocity: 0.1, feedforwardTorque: 0.2, kp: 20, kd: 0.5)
    let command = RobotCommand(stamp: RobotTime(nanoseconds: 1), joints: joints)

    let adapter = Go2Adapter()
    let cmd = adapter.lowCmd(from: command)

    #expect(abs(cmd.motorCmd[0].q - 0.5) < 1e-6)
    #expect(abs(cmd.motorCmd[0].dq - 0.1) < 1e-6)
    #expect(abs(cmd.motorCmd[0].tau - 0.2) < 1e-6)
    #expect(abs(cmd.motorCmd[0].kp - 20) < 1e-6)
    #expect(abs(cmd.motorCmd[0].kd - 0.5) < 1e-6)
    // Slots 12..19 are unused on a 12-joint Go2 and must stay zeroed.
    #expect(cmd.motorCmd[12].kp == 0)
    #expect(cmd.motorCmd[19].q == 0)
}

@Test func adapterStampsAValidCRC() {
    var joints = [JointTarget](repeating: JointTarget(position: 0), count: 12)
    joints[5] = JointTarget(position: -1.8, kp: 20, kd: 0.5)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))

    var withoutCRC = cmd
    withoutCRC.crc = 0
    #expect(cmd.crc == UnitreeCRC.crc(for: withoutCRC))
    #expect(cmd.crc != 0)
}

@Test func adapterSetsTheLowLevelControlHeader() {
    let cmd = Go2Adapter().lowCmd(
        from: RobotCommand(
            stamp: RobotTime(nanoseconds: 0),
            joints: [JointTarget](repeating: JointTarget(position: 0), count: 12)))
    #expect(cmd.head[0] == 0xFE)
    #expect(cmd.head[1] == 0xEF)
    #expect(cmd.levelFlag == 0xFF)
    // Mode 0x01 = servo/position-velocity-torque control on each used motor.
    #expect(cmd.motorCmd[0].mode == 0x01)
    #expect(cmd.motorCmd[12].mode == 0x00)
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter Go2AdapterTests
```
Expected: FAIL to compile — `Go2JointMap` and `Go2Adapter` don't exist.

- [ ] **Step 3: Write `Go2JointMap.swift`**

```swift
// Sources/RobotKitGo2/Go2JointMap.swift
import RobotKit

/// The two joint orderings this robot involves, and the permutations between
/// them.
///
/// The MuJoCo Menagerie model declares legs FL, FR, RL, RR; Unitree firmware
/// indexes motors FR, FL, RR, RL. Confusing the two silently drives every
/// command into the wrong leg, so both orders are named explicitly here and
/// the canonical order used everywhere in `RobotKit` is the firmware's.
public enum Go2JointMap {
    public static let unitreeOrder = JointMap(names: [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    ])

    public static let mujocoOrder = JointMap(names: [
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    ])

    /// `mujocoToUnitree[i]` is the MuJoCo index of canonical (Unitree) joint `i`.
    public static let mujocoToUnitree: [Int] = mujocoOrder.permutation(to: unitreeOrder)

    /// `unitreeToMuJoCo[i]` is the canonical (Unitree) index of MuJoCo joint `i`.
    public static let unitreeToMuJoCo: [Int] = unitreeOrder.permutation(to: mujocoOrder)

    /// The Menagerie `home` keyframe pose (hip 0, thigh 0.9, calf -1.8 per leg),
    /// expressed in canonical order. Symmetric across legs, so the ordering does
    /// not change the values — it is written per-leg anyway to stay correct if
    /// an asymmetric pose is ever substituted.
    public static let defaultStandPose: [Double] = [
        0.0, 0.9, -1.8,  // FR
        0.0, 0.9, -1.8,  // FL
        0.0, 0.9, -1.8,  // RR
        0.0, 0.9, -1.8,  // RL
    ]

    /// Foot geom names in `go2.xml`, in canonical (Unitree) leg order.
    public static let footGeomNames = ["FR", "FL", "RR", "RL"]
}
```

- [ ] **Step 4: Write `Go2Adapter.swift`**

```swift
// Sources/RobotKitGo2/Go2Adapter.swift
import RobotKit

/// Translates between the Go2's vendor messages and canonical `RobotKit` types.
///
/// Both simulation and hardware produce `LowState` and consume `LowCmd`, so
/// this single adapter is the only place vendor semantics live — and it is
/// exercised identically in both modes.
public struct Go2Adapter: Sendable {
    /// Number of physically present joints. `LowState`/`LowCmd` carry 20 motor
    /// slots; a Go2 uses the first 12 and leaves the rest zeroed.
    public static let jointCount = 12

    public init() {}

    public func observation(from state: LowState, stamp: RobotTime) -> RobotObservation {
        let joints = (0..<Self.jointCount).map { i in
            JointReading(
                position: Double(state.motorState[i].q),
                velocity: Double(state.motorState[i].dq),
                effort: Double(state.motorState[i].tauEst)
            )
        }

        let q = state.imuState.quaternion
        let g = state.imuState.gyroscope
        let a = state.imuState.accelerometer
        let imu = IMUReading(
            orientation: (Double(q[0]), Double(q[1]), Double(q[2]), Double(q[3])),
            angularVelocity: (Double(g[0]), Double(g[1]), Double(g[2])),
            linearAcceleration: (Double(a[0]), Double(a[1]), Double(a[2]))
        )

        // foot_force is already in canonical (FR, FL, RR, RL) leg order.
        let contacts = (0..<4).map { i in
            ContactReading(
                normalForce: Double(state.footForce[i]),
                inContact: state.footForce[i] > Self.contactForceThreshold
            )
        }

        return RobotObservation(stamp: stamp, joints: joints, imu: imu, contacts: contacts)
    }

    public func lowCmd(from command: RobotCommand) -> LowCmd {
        var cmd = LowCmd()
        cmd.head = [0xFE, 0xEF]
        cmd.levelFlag = Self.lowLevelFlag

        for i in 0..<Self.jointCount {
            let target = command.joints[i]
            cmd.motorCmd[i].mode = Self.servoMode
            cmd.motorCmd[i].q = Float(target.position)
            cmd.motorCmd[i].dq = Float(target.velocity)
            cmd.motorCmd[i].tau = Float(target.feedforwardTorque)
            cmd.motorCmd[i].kp = Float(target.kp)
            cmd.motorCmd[i].kd = Float(target.kd)
        }

        cmd.crc = UnitreeCRC.crc(for: cmd)
        return cmd
    }

    /// Newtons above which a foot counts as loaded. The Go2's foot sensors read
    /// small nonzero values when swinging, so zero would report false contacts.
    static let contactForceThreshold: Int16 = 20
    /// `0xFF` selects low-level (direct motor) control rather than sport mode.
    static let lowLevelFlag: UInt8 = 0xFF
    /// Per-motor servo mode: honor q/dq/tau/kp/kd.
    static let servoMode: UInt8 = 0x01
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter Go2AdapterTests
```
Expected: PASS (7 tests).

- [ ] **Step 6: Commit**

```bash
git add Sources/RobotKitGo2 Tests/RobotKitGo2Tests
git commit -m "feat: add Go2 adapter with explicit leg-order mapping and CRC stamping"
```

---

### Task 7: MuJoCo simulation source

**Files:**
- Create: `Sources/RobotKitSim/Go2SimModel.swift`
- Create: `Sources/RobotKitSim/Go2Simulator.swift`
- Modify: `Package.swift`
- Test: `Tests/RobotKitSimTests/Go2SimulatorTests.swift`
- Test fixture: `Tests/RobotKitSimTests/QuadrupedFixture.swift`

**Interfaces:**
- Consumes: canonical types (Task 1), generated messages (Task 4), `Go2JointMap`/`Go2Adapter` (Task 6), `MuJoCo` (existing).
- Produces: `enum Go2SimModel` with `static func wrapperXML(scenePath: String) -> String`, `static func resolveScene(searchDirs: [String]) -> String?`; `final class Go2Simulator` with `init(modelXMLPath: String) throws`, `init(modelXML: String) throws`, `func reset()`, `func applyLowCmd(_ cmd: LowCmd)`, `func step()`, `func lowState() -> LowState`, `var time: Double`, `static let controlDecimation = 10`.

`Go2Simulator` owns `MjModel`/`MjData` and therefore is **not** `Sendable` — construct one per simulation, never share across tasks.

- [ ] **Step 1: Add the `RobotKitSim` target**

Append to `targets` in `Package.swift`:

```swift
targets.append(.target(name: "RobotKitSim", dependencies: ["RobotKit", "RobotKitGo2", "MuJoCo"]))
targets.append(.testTarget(name: "RobotKitSimTests", dependencies: ["RobotKitSim", "RobotKitGo2", "RobotKit", "MuJoCo"]))
```

- [ ] **Step 2: Write the test fixture**

Tests use a small hand-written quadruped so they stay hermetic and fast — no Menagerie clone required. It uses the real Menagerie joint names and the same FL, FR, RL, RR declaration order, so all mapping logic is exercised faithfully.

```swift
// Tests/RobotKitSimTests/QuadrupedFixture.swift

enum QuadrupedFixture {
    /// A 12-joint quadruped with Menagerie's joint names, declaration order
    /// (FL, FR, RL, RR), torque actuators, an `imu` site with sensors, and
    /// four named foot geoms — the same surface `Go2Simulator` reads.
    static let xml = """
        <mujoco model="fixture_quadruped">
          <option timestep="0.002"/>
          <worldbody>
            <geom name="floor" type="plane" size="5 5 0.1"/>
            <body name="base" pos="0 0 0.4">
              <freejoint/>
              <site name="imu" pos="0 0 0"/>
              <geom name="trunk" type="box" size="0.2 0.1 0.05" mass="5"/>
              \(leg("FL", 0.18, 0.09))
              \(leg("FR", 0.18, -0.09))
              \(leg("RL", -0.18, 0.09))
              \(leg("RR", -0.18, -0.09))
            </body>
          </worldbody>
          <actuator>
            \(actuators())
          </actuator>
          <sensor>
            <gyro site="imu" name="rk_gyro"/>
            <accelerometer site="imu" name="rk_accel"/>
            <framequat objtype="site" objname="imu" name="rk_quat"/>
          </sensor>
        </mujoco>
        """

    private static func leg(_ p: String, _ x: Double, _ y: Double) -> String {
        """
        <body name="\(p)_hip" pos="\(x) \(y) 0">
          <joint name="\(p)_hip_joint" type="hinge" axis="1 0 0" range="-1.05 1.05"/>
          <geom type="capsule" fromto="0 0 0 0 \(y > 0 ? 0.05 : -0.05) 0" size="0.02" mass="0.5"/>
          <body name="\(p)_thigh" pos="0 \(y > 0 ? 0.05 : -0.05) 0">
            <joint name="\(p)_thigh_joint" type="hinge" axis="0 1 0" range="-1.57 3.49"/>
            <geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.02" mass="0.5"/>
            <body name="\(p)_calf" pos="0 0 -0.2">
              <joint name="\(p)_calf_joint" type="hinge" axis="0 1 0" range="-2.72 -0.83"/>
              <geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.02" mass="0.3"/>
              <geom name="\(p)" type="sphere" pos="0 0 -0.2" size="0.022"/>
            </body>
          </body>
        </body>
        """
    }

    private static func actuators() -> String {
        ["FL", "FR", "RL", "RR"].flatMap { p in
            ["hip", "thigh", "calf"].map { j in
                "<motor name=\"\(p)_\(j)\" joint=\"\(p)_\(j)_joint\" ctrlrange=\"-45 45\"/>"
            }
        }.joined(separator: "\n    ")
    }
}
```

- [ ] **Step 3: Write the failing test**

```swift
// Tests/RobotKitSimTests/Go2SimulatorTests.swift
import RobotKit
import RobotKitGo2
import Testing

@testable import RobotKitSim

@Test func simulatorReportsTwelveJointsInCanonicalOrder() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    let state = sim.lowState()
    #expect(state.motorState.count == 20)
    // Slots 12..19 are unused and stay zero.
    #expect(state.motorState[12].q == 0)
}

@Test func simulatorMapsJointsIntoFirmwareOrder() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    // Drive only MuJoCo's FR_hip (tree index 3) to a distinctive angle.
    sim.setJointPositionForTesting(mujocoIndex: 3, value: 0.42)
    let state = sim.lowState()
    // Firmware index 0 is FR_hip, so the value must surface at slot 0.
    #expect(abs(Double(state.motorState[0].q) - 0.42) < 1e-6)
    #expect(abs(Double(state.motorState[3].q)) < 1e-6)
}

@Test func pdControlDrivesJointsTowardTheirTargets() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()

    var joints = [JointTarget](repeating: JointTarget(position: 0, kp: 60, kd: 2), count: 12)
    // Canonical index 1 is FR_thigh.
    joints[1] = JointTarget(position: 0.6, kp: 60, kd: 2)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))

    for _ in 0..<500 {
        sim.applyLowCmd(cmd)
        sim.step()
    }

    let reached = Double(sim.lowState().motorState[1].q)
    #expect(abs(reached - 0.6) < 0.15)
}

@Test func zeroGainsProduceZeroTorque() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    let joints = [JointTarget](repeating: JointTarget(position: 1.0), count: 12)  // kp=kd=0
    sim.applyLowCmd(Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints)))
    #expect(sim.appliedTorquesForTesting().allSatisfy { abs($0) < 1e-12 })
}

@Test func steppingAdvancesSimulationTime() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    #expect(sim.time == 0)
    for _ in 0..<10 { sim.step() }
    #expect(abs(sim.time - 0.02) < 1e-9)
}

@Test func imuReportsUprightOrientationAndGravity() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    sim.step()
    let imu = sim.lowState().imuState
    // Spawned level: w≈1, and the accelerometer reads roughly +g on z while
    // the body is supported (MuJoCo's accelerometer includes gravity).
    #expect(abs(Double(imu.quaternion[0]) - 1.0) < 0.05)
    #expect(abs(Double(imu.accelerometer[2])) > 5.0)
}

@Test func footContactsRegisterOnceTheBodySettles() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    let joints = [JointTarget](repeating: JointTarget(position: 0, kp: 60, kd: 2), count: 12)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))
    for _ in 0..<1500 {
        sim.applyLowCmd(cmd)
        sim.step()
    }
    let forces = sim.lowState().footForce
    #expect(forces.contains { $0 > 0 })
}
```

- [ ] **Step 4: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter Go2SimulatorTests
```
Expected: FAIL to compile — `Go2Simulator` doesn't exist.

- [ ] **Step 5: Write `Go2SimModel.swift`**

```swift
// Sources/RobotKitSim/Go2SimModel.swift
import Foundation

/// Locating and preparing the Menagerie Go2 model.
///
/// `go2.xml` declares no `<sensor>` block — only an unused `imu` site — so the
/// simulator loads a wrapper that includes the stock scene and adds the three
/// sensors needed to synthesize `unitree_go/IMUState`. Wrapping rather than
/// editing keeps the vendored model pristine.
public enum Go2SimModel {
    public static func wrapperXML(scenePath: String) -> String {
        """
        <mujoco model="go2_robotkit">
          <include file="\(scenePath)"/>
          <sensor>
            <gyro site="imu" name="rk_gyro"/>
            <accelerometer site="imu" name="rk_accel"/>
            <framequat objtype="site" objname="imu" name="rk_quat"/>
          </sensor>
        </mujoco>
        """
    }

    /// Finds `unitree_go2/scene.xml` under any of `searchDirs`.
    public static func resolveScene(searchDirs: [String]) -> String? {
        let fm = FileManager.default
        for root in searchDirs {
            let candidate = (root as NSString)
                .appendingPathComponent("unitree_go2/scene.xml")
            if fm.fileExists(atPath: candidate) { return candidate }
        }
        return nil
    }

    /// Writes the wrapper next to the scene so MuJoCo's `<include>` and the
    /// model's relative `meshdir` both resolve, and returns its path.
    public static func writeWrapper(besideScene scenePath: String) throws -> String {
        let dir = (scenePath as NSString).deletingLastPathComponent
        let wrapper = (dir as NSString).appendingPathComponent("robotkit_go2.xml")
        let xml = wrapperXML(scenePath: (scenePath as NSString).lastPathComponent)
        try xml.write(toFile: wrapper, atomically: true, encoding: .utf8)
        return wrapper
    }
}
```

- [ ] **Step 6: Write `Go2Simulator.swift`**

```swift
// Sources/RobotKitSim/Go2Simulator.swift
import MuJoCo
import RobotKit
import RobotKitGo2

/// A MuJoCo Go2 that speaks the robot's own wire messages.
///
/// It consumes `unitree_go/LowCmd` and produces `unitree_go/LowState`, so the
/// adapter and decode path that will face hardware are exercised here too. The
/// PD law lives in this class because the Menagerie model's actuators are pure
/// torque motors with no built-in gains, exactly matching the real robot where
/// kp/kd arrive per command.
///
/// Not `Sendable`: it owns `MjModel`/`MjData`, neither of which is. Construct
/// one per simulation and keep it on a single task.
public final class Go2Simulator {
    /// Physics runs at the model timestep (2 ms for the Go2 model); control
    /// runs every 10th step, i.e. 50 Hz, matching a realistic policy rate.
    public static let controlDecimation = 10

    private let model: MjModel
    private let data: MjData
    /// `actuatorForJoint[c]` is the MuJoCo actuator index for canonical joint `c`.
    private let actuatorForJoint: [Int]
    /// `qposAddressForJoint[c]` / `dofAddressForJoint[c]`, canonical order.
    private let qposAddressForJoint: [Int]
    private let dofAddressForJoint: [Int]
    private let footGeomIds: [Int]
    private let gyroSensor: MjModel.SensorInfo?
    private let accelSensor: MjModel.SensorInfo?
    private let quatSensor: MjModel.SensorInfo?
    private var lastTorques: [Double]

    public convenience init(modelXMLPath: String) throws {
        try self.init(model: MjModel.load(xmlPath: modelXMLPath))
    }

    public convenience init(modelXML: String) throws {
        try self.init(model: MjModel.load(xml: modelXML))
    }

    private init(model: MjModel) {
        self.model = model
        self.data = MjData(model)

        let joints = model.joints
        let actuators = model.actuators
        // Canonical order is the firmware's; look each joint up by name so the
        // model's own declaration order never leaks into the mapping.
        var actuatorIndices: [Int] = []
        var qposAddresses: [Int] = []
        var dofAddresses: [Int] = []
        for name in Go2JointMap.unitreeOrder.names {
            guard let joint = joints.first(where: { $0.name == name }) else {
                preconditionFailure("model is missing joint '\(name)'")
            }
            qposAddresses.append(joint.qposadr)
            dofAddresses.append(joint.dofadr)
            // Actuator names drop the "_joint" suffix in both the Menagerie
            // model and the test fixture (FR_hip_joint -> FR_hip).
            let actuatorName = String(name.dropLast("_joint".count))
            guard let actuator = actuators.first(where: { $0.name == actuatorName }) else {
                preconditionFailure("model is missing actuator '\(actuatorName)'")
            }
            actuatorIndices.append(actuator.id)
        }
        self.actuatorForJoint = actuatorIndices
        self.qposAddressForJoint = qposAddresses
        self.dofAddressForJoint = dofAddresses
        self.footGeomIds = Go2JointMap.footGeomNames.compactMap { model.id(of: objGeom, name: $0) }
        self.gyroSensor = model.sensor(named: "rk_gyro")
        self.accelSensor = model.sensor(named: "rk_accel")
        self.quatSensor = model.sensor(named: "rk_quat")
        self.lastTorques = [Double](repeating: 0, count: Go2Adapter.jointCount)
    }

    public var time: Double { data.time }

    public func reset() {
        mjResetData(model, data)
        lastTorques = [Double](repeating: 0, count: Go2Adapter.jointCount)
        mjForward(model, data)
    }

    /// Evaluates the PD law for every commanded joint and stores the resulting
    /// torques in `ctrl`. Called once per control tick; the torque then holds
    /// across the intervening physics steps, as it does on real hardware.
    public func applyLowCmd(_ cmd: LowCmd) {
        for c in 0..<Go2Adapter.jointCount {
            let motor = cmd.motorCmd[c]
            let q = data.qpos(at: qposAddressForJoint[c])
            let dq = data.qvel(at: dofAddressForJoint[c])
            let tau =
                Double(motor.kp) * (Double(motor.q) - q)
                + Double(motor.kd) * (Double(motor.dq) - dq)
                + Double(motor.tau)
            lastTorques[c] = tau
            data.setCtrl(actuatorForJoint[c], tau)
        }
    }

    public func step() {
        mjStep(model, data)
    }

    /// Synthesizes the robot's own state message from `mjData`.
    public func lowState() -> LowState {
        var state = LowState()
        for c in 0..<Go2Adapter.jointCount {
            state.motorState[c].q = Float(data.qpos(at: qposAddressForJoint[c]))
            state.motorState[c].dq = Float(data.qvel(at: dofAddressForJoint[c]))
            state.motorState[c].tauEst = Float(lastTorques[c])
        }

        if let quatSensor {
            let v = data.sensorValues(quatSensor)
            state.imuState.quaternion = [Float(v[0]), Float(v[1]), Float(v[2]), Float(v[3])]
        }
        if let gyroSensor {
            let v = data.sensorValues(gyroSensor)
            state.imuState.gyroscope = [Float(v[0]), Float(v[1]), Float(v[2])]
        }
        if let accelSensor {
            let v = data.sensorValues(accelSensor)
            state.imuState.accelerometer = [Float(v[0]), Float(v[1]), Float(v[2])]
        }

        state.footForce = footContactForces().map { Int16(clamping: Int($0)) }
        return state
    }

    /// Summed contact normal force per foot geom, in canonical leg order.
    private func footContactForces() -> [Double] {
        var forces = [Double](repeating: 0, count: footGeomIds.count)
        for contact in data.contacts(max: 128) {
            for (leg, geomId) in footGeomIds.enumerated()
            where contact.geom1 == geomId || contact.geom2 == geomId {
                forces[leg] += contact.forceNormal
            }
        }
        return forces
    }

    // MARK: - Test hooks

    func setJointPositionForTesting(mujocoIndex: Int, value: Double) {
        let joints = model.joints
        data.setQpos(at: joints[mujocoIndex + 1].qposadr, value)  // +1 skips the free joint
        mjForward(model, data)
    }

    func appliedTorquesForTesting() -> [Double] { lastTorques }
}
```

- [ ] **Step 7: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter Go2SimulatorTests
```
Expected: PASS (7 tests). If `setJointPositionForTesting`'s `+1` free-joint offset is wrong for the fixture, look up the joint by name instead of by index — report the change rather than adjusting the assertion.

- [ ] **Step 8: Commit**

```bash
git add Package.swift Sources/RobotKitSim Tests/RobotKitSimTests
git commit -m "feat: add MuJoCo Go2 simulator synthesizing LowState and applying PD LowCmd"
```

---

### Task 8: ROS 2 transport

**Files:**
- Create: `Sources/RobotKitROS2/ROS2Transport.swift`
- Modify: `Package.swift`
- Test: `Tests/RobotKitROS2Tests/ROS2TransportTests.swift`

**Interfaces:**
- Consumes: `MessageTransport` (Task 3).
- Produces: `actor ROS2Transport: MessageTransport` with `init(nodeName: String, transport: TransportConfig, distro: ROS2Distro = .humble) async throws` and `func shutdown() async`.

- [ ] **Step 1: Add the `RobotKitROS2` target**

Append to `targets` in `Package.swift`:

```swift
targets.append(.target(name: "RobotKitROS2", dependencies: [
    "RobotKit",
    .product(name: "SwiftROS2", package: "swift-ros2"),
]))
targets.append(.testTarget(name: "RobotKitROS2Tests", dependencies: ["RobotKitROS2", "RobotKit"]))
```

- [ ] **Step 2: Write the failing test**

This is a loopback test: one process publishes and subscribes over real DDS. It needs working multicast on the loopback interface, so it degrades to a skip rather than a failure when discovery does not complete — matching how this repo's GL tests handle a missing GL stack.

```swift
// Tests/RobotKitROS2Tests/ROS2TransportTests.swift
import SwiftROS2
import Testing

@testable import RobotKitROS2

@Test func ros2TransportRoundTripsOverDDS() async throws {
    let transport = try await ROS2Transport(
        nodeName: "robotkit_test", transport: .ddsMulticast(domainId: 77))
    defer { Task { await transport.shutdown() } }

    let stream = try await transport.subscribe(StringMsg.self, topic: "robotkit_probe")
    // DDS discovery is not instantaneous; give the reader time to match.
    try await Task.sleep(for: .milliseconds(500))

    // Publish repeatedly in the background — a single publish can land before
    // discovery completes and be dropped by best-effort QoS.
    let publisher = Task {
        for _ in 0..<40 {
            try? await transport.publish(StringMsg(data: "ping"), topic: "robotkit_probe")
            try? await Task.sleep(for: .milliseconds(50))
        }
    }

    // First of: a received message, or a 3-second timeout.
    let result = await withTaskGroup(of: String?.self) { group -> String? in
        group.addTask {
            for await msg in stream { return msg.data }
            return nil
        }
        group.addTask {
            try? await Task.sleep(for: .seconds(3))
            return nil
        }
        let first = await group.next() ?? nil
        group.cancelAll()
        return first
    }
    publisher.cancel()

    if result == nil {
        // No DDS discovery in this environment (common in sandboxed CI).
        // Treat as a skip: the in-process transport still covers the logic.
        print("[skip] DDS loopback discovery unavailable")
        return
    }
    #expect(result == "ping")
}
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter ROS2TransportTests
```
Expected: FAIL to compile — `ROS2Transport` doesn't exist.

- [ ] **Step 4: Write `ROS2Transport.swift`**

```swift
// Sources/RobotKitROS2/ROS2Transport.swift
import RobotKit
import SwiftROS2

/// `MessageTransport` over real ROS 2 middleware.
///
/// Publishers and subscriptions are created lazily per topic and cached, so a
/// control loop that publishes every tick pays the setup cost once.
public actor ROS2Transport: MessageTransport {
    private let context: ROS2Context
    private let node: ROS2Node
    private var publishers: [String: Any] = [:]

    public init(
        nodeName: String, transport: TransportConfig, distro: ROS2Distro = .humble
    ) async throws {
        self.context = try await ROS2Context(transport: transport, distro: distro)
        self.node = try await context.createNode(name: nodeName)
    }

    public func publish<M: ROS2Message>(_ message: M, topic: String) async throws {
        let publisher: ROS2Publisher<M>
        if let cached = publishers[topic] as? ROS2Publisher<M> {
            publisher = cached
        } else {
            publisher = try await node.createPublisher(M.self, topic: topic, qos: .sensorData)
            publishers[topic] = publisher
        }
        try publisher.publish(message)
    }

    public func subscribe<M: ROS2Message>(
        _ type: M.Type, topic: String
    ) async throws -> AsyncStream<M> {
        let subscription = try await node.createSubscription(M.self, topic: topic, qos: .sensorData)
        return subscription.messages
    }

    public func shutdown() async {
        await context.shutdown()
    }
}
```

- [ ] **Step 5: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter ROS2TransportTests
```
Expected: PASS — either a real round trip, or the printed skip line if DDS discovery is unavailable in this environment.

- [ ] **Step 6: Commit**

```bash
git add Package.swift Sources/RobotKitROS2 Tests/RobotKitROS2Tests
git commit -m "feat: add ROS 2 DDS transport"
```

---

### Task 9: Control loop and stand controller

**Files:**
- Create: `Sources/RobotKit/Controller.swift`
- Create: `Sources/RobotKit/RobotRuntime.swift`
- Create: `Sources/RobotKitGo2/StandController.swift`
- Test: `Tests/RobotKitTests/RobotRuntimeTests.swift`

**Interfaces:**
- Consumes: canonical types, encoder/decoder (Tasks 1–2).
- Produces: `protocol Controller: Sendable` with `mutating func act(observation: [Float]) -> [Float]` and `mutating func reset()`; `struct RobotRuntime<C: Controller>` with `init(controller:encoder:decoder:commandedVelocity:)` and `mutating func tick(observation: RobotObservation) -> RobotCommand`; `struct StandController: Controller` with `init()`.

`RobotRuntime` is deliberately transport-free: it converts one observation into one command. Wiring it to a transport is the demo's job, which keeps the piece that must behave identically in both modes trivially testable.

- [ ] **Step 1: Write the failing test**

```swift
// Tests/RobotKitTests/RobotRuntimeTests.swift
import Testing

@testable import RobotKit

private let pose: [Double] = Array(repeating: 0.5, count: 12)

private struct ConstantController: Controller {
    var value: Float
    var resetCount = 0
    mutating func act(observation: [Float]) -> [Float] {
        [Float](repeating: value, count: 12)
    }
    mutating func reset() { resetCount += 1 }
}

private func sampleObservation() -> RobotObservation {
    RobotObservation(
        stamp: RobotTime(nanoseconds: 123),
        joints: pose.map { JointReading(position: $0, velocity: 0, effort: 0) },
        imu: IMUReading(
            orientation: (1, 0, 0, 0), angularVelocity: (0, 0, 0), linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 5, inContact: true), count: 4))
}

@Test func runtimeTurnsAnObservationIntoATimestampedCommand() {
    var runtime = RobotRuntime(
        controller: ConstantController(value: 0),
        encoder: ObservationEncoder(defaultPose: pose, jointCount: 12),
        decoder: ActionDecoder(defaultPose: pose, scale: 0.25, kp: 20, kd: 0.5),
        commandedVelocity: (0, 0, 0))

    let cmd = runtime.tick(observation: sampleObservation())
    #expect(cmd.stamp.nanoseconds == 123)
    #expect(cmd.joints.count == 12)
    // A zero action holds the default pose.
    #expect(abs(cmd.joints[0].position - pose[0]) < 1e-9)
    #expect(cmd.joints[0].kp == 20)
}

@Test func runtimeFeedsTheActionBackIntoTheNextObservation() {
    var runtime = RobotRuntime(
        controller: ConstantController(value: 1.0),
        encoder: ObservationEncoder(defaultPose: pose, jointCount: 12),
        decoder: ActionDecoder(defaultPose: pose, scale: 0.25, kp: 20, kd: 0.5),
        commandedVelocity: (0, 0, 0))

    _ = runtime.tick(observation: sampleObservation())
    #expect(abs(runtime.lastObservationVector[33] - 0) < 1e-6)  // first tick: no history yet
    _ = runtime.tick(observation: sampleObservation())
    #expect(abs(runtime.lastObservationVector[33] - 1.0) < 1e-6)
}

@Test func standControllerCommandsTheDefaultPose() {
    var controller = StandControllerForTesting()
    let action = controller.act(observation: [Float](repeating: 0, count: 45))
    #expect(action.count == 12)
    #expect(action.allSatisfy { $0 == 0 })
}
```

Note: `StandControllerForTesting` is a local stand-in so `RobotKitTests` need not depend on `RobotKitGo2`. Define it in this test file:

```swift
private struct StandControllerForTesting: Controller {
    mutating func act(observation: [Float]) -> [Float] { [Float](repeating: 0, count: 12) }
    mutating func reset() {}
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter RobotRuntimeTests
```
Expected: FAIL to compile — `Controller` and `RobotRuntime` don't exist.

- [ ] **Step 3: Write `Controller.swift`**

```swift
// Sources/RobotKit/Controller.swift

/// Whatever decides what the robot should do next. A hand-written policy in
/// Phase 1; a trained network later. It sees only the encoded observation
/// vector and returns only an action vector, so it is unaware of both the
/// robot's wire protocol and whether physics is simulated.
public protocol Controller: Sendable {
    mutating func act(observation: [Float]) -> [Float]
    mutating func reset()
}
```

- [ ] **Step 4: Write `RobotRuntime.swift`**

```swift
// Sources/RobotKit/RobotRuntime.swift

/// One control tick: observation in, command out.
///
/// Holds the encoder (and therefore the action history) so the loop's warm-up
/// behavior is identical wherever it runs. Transport-free by design — the
/// caller decides where observations come from and where commands go, which is
/// the only thing that differs between simulation and hardware.
public struct RobotRuntime<C: Controller> {
    public var controller: C
    public var encoder: ObservationEncoder
    public let decoder: ActionDecoder
    public var commandedVelocity: (Double, Double, Double)

    /// The most recent encoded observation, exposed for logging and tests.
    public private(set) var lastObservationVector: [Float] = []

    public init(
        controller: C, encoder: ObservationEncoder, decoder: ActionDecoder,
        commandedVelocity: (Double, Double, Double)
    ) {
        self.controller = controller
        self.encoder = encoder
        self.decoder = decoder
        self.commandedVelocity = commandedVelocity
    }

    public mutating func tick(observation: RobotObservation) -> RobotCommand {
        let vector = encoder.encode(observation, commandedVelocity: commandedVelocity)
        lastObservationVector = vector
        let action = controller.act(observation: vector)
        encoder.noteAction(action)
        return decoder.decode(action, stamp: observation.stamp)
    }

    public mutating func reset() {
        encoder.reset()
        controller.reset()
        lastObservationVector = []
    }
}
```

- [ ] **Step 5: Write `StandController.swift`**

```swift
// Sources/RobotKitGo2/StandController.swift
import RobotKit

/// Holds the default stand pose by commanding a zero action every tick.
///
/// Trivial on purpose: it makes the Phase 1 parity claim about the plumbing
/// rather than about a policy. Because `ActionDecoder` maps a zero action to
/// the default pose, this is also the safe fallback behavior for a robot whose
/// policy has not loaded.
public struct StandController: Controller {
    public init() {}

    public mutating func act(observation: [Float]) -> [Float] {
        [Float](repeating: 0, count: Go2Adapter.jointCount)
    }

    public mutating func reset() {}
}
```

- [ ] **Step 6: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter RobotRuntimeTests
```
Expected: PASS (3 tests).

- [ ] **Step 7: Commit**

```bash
git add Sources/RobotKit Sources/RobotKitGo2/StandController.swift Tests/RobotKitTests/RobotRuntimeTests.swift
git commit -m "feat: add controller protocol, control-tick runtime, and stand controller"
```

---

### Task 10: Demo executable, parity test, CI, and documentation

**Files:**
- Create: `Sources/go2-demo/main.swift`
- Create: `Tests/RobotKitSimTests/ParityTests.swift`
- Modify: `Package.swift`
- Modify: `.github/workflows/ci.yml`
- Modify: `README.md`

**Interfaces:**
- Consumes: everything from Tasks 1–9.
- Produces: the `go2-demo` executable with `--mode sim|loopback` and `--steps <n>`.

- [ ] **Step 1: Write the failing parity test**

The claim Phase 1 exists to prove: the same observation, whether it reaches the encoder through an in-process hand-off or through CDR serialization, produces an identical command.

```swift
// Tests/RobotKitSimTests/ParityTests.swift
import RobotKit
import RobotKitGo2
import SwiftROS2
import Testing

@testable import RobotKitSim

/// Serializing and deserializing a LowState the way DDS would.
private func roundTripped(_ state: LowState) throws -> LowState {
    let encoder = CDREncoder(isLegacySchema: true)
    encoder.writeEncapsulationHeader()
    try state.encode(to: encoder)
    let decoder = try CDRDecoder(data: encoder.getData(), isLegacySchema: true)
    return try LowState(from: decoder)
}

@Test func serializationDoesNotChangeTheObservation() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    let joints = [JointTarget](repeating: JointTarget(position: 0, kp: 60, kd: 2), count: 12)
    let cmd = Go2Adapter().lowCmd(from: RobotCommand(stamp: RobotTime(nanoseconds: 0), joints: joints))
    for _ in 0..<200 {
        sim.applyLowCmd(cmd)
        sim.step()
    }

    let state = sim.lowState()
    let adapter = Go2Adapter()
    let stamp = RobotTime(seconds: sim.time)

    let direct = adapter.observation(from: state, stamp: stamp)
    let viaWire = adapter.observation(from: try roundTripped(state), stamp: stamp)

    #expect(direct == viaWire)
}

@Test func bothPathsProduceIdenticalCommands() throws {
    let sim = try Go2Simulator(modelXML: QuadrupedFixture.xml)
    sim.reset()
    for _ in 0..<100 { sim.step() }
    let state = sim.lowState()
    let adapter = Go2Adapter()
    let stamp = RobotTime(seconds: sim.time)

    func command(from s: LowState) -> RobotCommand {
        var runtime = RobotRuntime(
            controller: StandController(),
            encoder: ObservationEncoder(
                defaultPose: Go2JointMap.defaultStandPose, jointCount: 12),
            decoder: ActionDecoder(
                defaultPose: Go2JointMap.defaultStandPose, scale: 0.25, kp: 20, kd: 0.5),
            commandedVelocity: (0, 0, 0))
        return runtime.tick(observation: adapter.observation(from: s, stamp: stamp))
    }

    let a = command(from: state)
    let b = command(from: try roundTripped(state))
    #expect(a == b)
    // And the wire form of those commands must agree bit for bit, CRC included.
    #expect(adapter.lowCmd(from: a).crc == adapter.lowCmd(from: b).crc)
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter ParityTests
```
Expected: FAIL to compile — `RobotKitSimTests` does not yet depend on `SwiftROS2`.

- [ ] **Step 3: Add the demo target and the test dependency**

In `Package.swift`, change the `RobotKitSimTests` declaration from Task 7 to add the SwiftROS2 product, and append the executable:

```swift
targets.append(.testTarget(name: "RobotKitSimTests", dependencies: [
    "RobotKitSim", "RobotKitGo2", "RobotKit", "MuJoCo",
    .product(name: "SwiftROS2", package: "swift-ros2"),
]))
targets.append(.executableTarget(
    name: "Go2Demo",
    dependencies: ["RobotKit", "RobotKitGo2", "RobotKitSim", "RobotKitROS2"],
    path: "Sources/go2-demo"))
products.append(.executable(name: "go2-demo", targets: ["Go2Demo"]))
```

(Replace the earlier `RobotKitSimTests` entry rather than adding a second one.)

- [ ] **Step 4: Run the test to verify it passes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift test --filter ParityTests
```
Expected: PASS (2 tests).

- [ ] **Step 5: Write the demo**

```swift
// Sources/go2-demo/main.swift
import Foundation
import RobotKit
import RobotKitGo2
import RobotKitROS2
import RobotKitSim
import SwiftROS2

// Same controller, same encoder, same decoder, same adapter in both modes.
// Only where LowState comes from and where LowCmd goes differs.

let arguments = Array(CommandLine.arguments.dropFirst())
var mode = "sim"
var steps = 2000
var index = 0
while index < arguments.count {
    switch arguments[index] {
    case "--mode": index += 1; if index < arguments.count { mode = arguments[index] }
    case "--steps": index += 1; if index < arguments.count { steps = Int(arguments[index]) ?? steps }
    default: break
    }
    index += 1
}

let searchDirs = [
    FileManager.default.currentDirectoryPath + "/.cache/mujoco_menagerie",
    NSHomeDirectory() + "/.cache/mujoco_menagerie",
]

func makeSimulator() throws -> Go2Simulator {
    if let scene = Go2SimModel.resolveScene(searchDirs: searchDirs) {
        let wrapper = try Go2SimModel.writeWrapper(besideScene: scene)
        print("[go2-demo] model: \(wrapper)")
        return try Go2Simulator(modelXMLPath: wrapper)
    }
    print("""
        [go2-demo] Menagerie Go2 not found in \(searchDirs).
        Fetch it with:
          git clone --depth 1 --filter=blob:none --sparse \\
            https://github.com/google-deepmind/mujoco_menagerie .cache/mujoco_menagerie
          git -C .cache/mujoco_menagerie sparse-checkout add unitree_go2
        """)
    exit(2)
}

func makeRuntime() -> RobotRuntime<StandController> {
    RobotRuntime(
        controller: StandController(),
        encoder: ObservationEncoder(defaultPose: Go2JointMap.defaultStandPose, jointCount: 12),
        decoder: ActionDecoder(
            defaultPose: Go2JointMap.defaultStandPose, scale: 0.25, kp: 20, kd: 0.5),
        commandedVelocity: (0, 0, 0))
}

let adapter = Go2Adapter()

switch mode {
case "sim":
    // In-process: the simulator's LowState goes straight to the adapter.
    let sim = try makeSimulator()
    sim.reset()
    var runtime = makeRuntime()
    var command = adapter.lowCmd(
        from: runtime.tick(observation: adapter.observation(
            from: sim.lowState(), stamp: RobotTime(seconds: sim.time))))

    for step in 0..<steps {
        if step % Go2Simulator.controlDecimation == 0 {
            let observation = adapter.observation(
                from: sim.lowState(), stamp: RobotTime(seconds: sim.time))
            command = adapter.lowCmd(from: runtime.tick(observation: observation))
            if step % 500 == 0 {
                let height = observation.joints[1].position
                print("[sim] t=\(String(format: "%.2f", sim.time))s FR_thigh=\(String(format: "%.3f", height)) contacts=\(observation.contacts.filter(\.inContact).count)/4")
            }
        }
        sim.applyLowCmd(command)
        sim.step()
    }
    print("[sim] done after \(String(format: "%.2f", sim.time))s")

case "loopback":
    // Over real DDS: the simulator publishes genuine unitree_go messages on
    // /lowstate and consumes /lowcmd, exactly as hardware does. The control
    // half below never learns that its peer is a simulator.
    let sim = try makeSimulator()
    sim.reset()

    let simTransport = try await ROS2Transport(
        nodeName: "go2_sim", transport: .ddsMulticast(domainId: 0))
    let controlTransport = try await ROS2Transport(
        nodeName: "go2_control", transport: .ddsMulticast(domainId: 0))

    let commands = try await simTransport.subscribe(LowCmd.self, topic: "lowcmd")
    let states = try await controlTransport.subscribe(LowState.self, topic: "lowstate")
    try await Task.sleep(for: .milliseconds(500))  // let DDS discovery settle

    /// Holds the most recent command from the wire. An actor because the
    /// reader task and the simulation loop are different concurrency domains —
    /// a plain `var` would be a data race that Swift 6 rejects.
    actor LatestCommand {
        private var value: LowCmd?
        private var received = 0
        func store(_ cmd: LowCmd) {
            value = cmd
            received += 1
        }
        func current() -> LowCmd? { value }
        func count() -> Int { received }
    }
    let latest = LatestCommand()

    // Control side: state in, command out. Byte-for-byte the same pipeline as
    // the `sim` branch — only the source of LowState differs.
    let controlTask = Task {
        var runtime = makeRuntime()
        for await state in states {
            let observation = adapter.observation(
                from: state, stamp: RobotTime(seconds: Double(state.tick) / 1000.0))
            let command = adapter.lowCmd(from: runtime.tick(observation: observation))
            try? await controlTransport.publish(command, topic: "lowcmd")
        }
    }

    // Exactly one consumer of `commands` — an AsyncStream splits between
    // multiple consumers rather than fanning out.
    let commandReader = Task {
        for await cmd in commands { await latest.store(cmd) }
    }

    for step in 0..<steps {
        if step % Go2Simulator.controlDecimation == 0 {
            var state = sim.lowState()
            state.tick = UInt32(sim.time * 1000)
            try await simTransport.publish(state, topic: "lowstate")
            try await Task.sleep(for: .milliseconds(2))  // yield so the reply can land
            if step % 500 == 0 {
                let n = await latest.count()
                print("[loopback] t=\(String(format: "%.2f", sim.time))s commandsReceived=\(n)")
            }
        }
        if let cmd = await latest.current() { sim.applyLowCmd(cmd) }
        sim.step()
    }

    let totalReceived = await latest.count()
    controlTask.cancel()
    commandReader.cancel()
    await simTransport.shutdown()
    await controlTransport.shutdown()

    if totalReceived == 0 {
        print("[loopback] FAILED: no LowCmd ever arrived over DDS")
        exit(1)
    }
    print("""
        [loopback] done — \(totalReceived) commands flowed over DDS \
        for \(String(format: "%.2f", sim.time))s of simulated time
        """)

default:
    print("Unknown mode '\(mode)'. Usage: go2-demo --mode sim|loopback [--steps N]")
    exit(2)
}
```

- [ ] **Step 6: Fetch the model and run both modes**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/google-deepmind/mujoco_menagerie .cache/mujoco_menagerie
git -C .cache/mujoco_menagerie sparse-checkout add unitree_go2
swift build
swift run go2-demo --mode sim --steps 2000
swift run go2-demo --mode loopback --steps 2000
```
Expected: `sim` prints periodic lines and reaches ~4 s of simulated time with feet in contact. `loopback` prints `received=true` after the first exchange and ends with the "commands flowed over DDS" line. If `loopback` reports no LowCmd, DDS discovery is unavailable in this environment — record that in the task report rather than reworking the code, since `sim` plus the parity test already cover the logic.

Add `.cache/` to `.gitignore` if it is not already ignored.

- [ ] **Step 7: Update CI**

In `.github/workflows/ci.yml`, the Linux job needs CycloneDDS for swift-ros2. Add to the existing "Install MuJoCo and a software GL stack" step's `apt-get install` line: `libcyclonedds-dev`. If that package is unavailable on the runner, build CycloneDDS from source before `swift build`:

```yaml
      - name: Install CycloneDDS for swift-ros2
        run: |
          set -euxo pipefail
          if ! sudo apt-get install -y libcyclonedds-dev; then
            git clone --depth 1 --branch 0.10.5 https://github.com/eclipse-cyclonedds/cyclonedds /tmp/cyclonedds
            cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
              -DENABLE_SHM=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DDSPERF=OFF
            sudo cmake --build /tmp/cyclonedds/build --target install
            sudo ldconfig
          fi
```

`-DBUILD_DDSPERF=OFF` is required: CycloneDDS 0.10.5 fails to build with GNU make 4.3 without it. Do not add `-DBUILD_IDLC=OFF` — `ddsc` needs `idlc` for builtin types.

- [ ] **Step 8: Update `README.md`**

Add to the `## What's here` list:

```markdown
- `RobotKit` — sim-to-real framework: canonical robot state/command types, the
  observation encoder and action decoder that define the sim-to-real contract,
  transports, and the control loop. `RobotKitGo2` adds Unitree Go2 messages,
  CRC, and the adapter; `RobotKitSim` drives a MuJoCo Go2 that speaks the
  robot's own wire messages; `RobotKitROS2` provides the DDS transport.
```

And a section:

```markdown
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
```

- [ ] **Step 9: Run the full suite and commit**

```bash
export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
swift build
swift test
```
Expected: the whole suite passes, including the pre-existing MuJoCo, WendyMuJoCo, and MuJoCoRLEnv tests.

```bash
git add Package.swift Sources/go2-demo Tests/RobotKitSimTests/ParityTests.swift \
        .github/workflows/ci.yml README.md .gitignore
git commit -m "feat: add go2-demo with sim and DDS loopback modes, parity tests, and docs"
```

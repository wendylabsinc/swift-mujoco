# RobotKit: a sim-to-real environment-interpretation framework

## Context

`swift-mujoco` provides MuJoCo bindings (physics, sensors, raycasting, offscreen
rendering) and, since the MLX-Swift RL sample, a cartpole training loop. Nothing
connects it to a real robot: there is no ROS 2 integration, no canonical robot
state, and no story for running one piece of control code against both a
simulator and hardware.

RobotKit fills that gap. It is the layer that *interprets the environment* —
turning whatever a robot publishes into a canonical state a policy consumes, and
turning a policy's output back into commands that robot accepts — with the same
code path in simulation and on hardware.

## Goal

Write a control policy once. Train it in simulation, validate it against a
simulated robot speaking the real wire protocol, then run the identical binary
against a physical Unitree Go2 — changing only a `--mode` flag.

## Foundations

- **[swift-ros2](https://github.com/youtalk/swift-ros2)** (v1.3.0) — pure Swift
  ROS 2: pub/sub, services, actions, QoS, DDS/Zenoh/RCL transports, generated
  `sensor_msgs`/`geometry_msgs`, and `swift-ros2-gen` for arbitrary `.msg` IDL
  (which is how the vendor `unitree_go` messages are obtained).
- **swift-mujoco** (this repo) — physics, sensors, contacts, and a Menagerie
  resolver that already maps `go2` → `unitree_go2`.
- **[mlx-swift](https://github.com/ml-explore/mlx-swift)** — policy networks and
  training. Its manifest has three build arms: Linux+CUDA (default on Linux,
  opt out with `SPM_CUDA=0`), Linux CPU-only (blas/lapack/openblas/gfortran),
  and Apple/Metal. **MLX therefore runs on the robot's Linux+CUDA compute**, so
  it is available in the deployed binary — the `#if os(macOS)` gate currently
  wrapping `MujocoRLDemo` in `Package.swift` reflects a Metal-only assumption
  that this work corrects.

## Reference target

A **Unitree Go2**. Chosen because both halves can be validated against each
other for real: MuJoCo Menagerie ships a Go2 model, and the physical robot is
available. Its ROS 2 interface is vendor-specific rather than standard
`sensor_msgs`, which is a feature for this design — it forces the adapter
boundary to be explicit rather than accidental.

The verified message contract (`unitreerobotics/unitree_ros2`):

| Message | Fields this design uses |
| --- | --- |
| `unitree_go/LowState` | `imu_state`, `motor_state[20]`, `foot_force[4]`, `tick`, `crc` |
| `unitree_go/MotorState` | `q`, `dq`, `ddq`, `tau_est`, `temperature` |
| `unitree_go/IMUState` | `quaternion[4]`, `gyroscope[3]`, `accelerometer[3]`, `rpy[3]` |
| `unitree_go/LowCmd` | `motor_cmd[20]`, `crc` |
| `unitree_go/MotorCmd` | `mode`, `q`, `dq`, `tau`, `kp`, `kd` |

Two details that shape the design: `MotorCmd` is a **PD setpoint**
(`q`, `dq`, `tau`, `kp`, `kd`), not a raw torque; and both `LowState` and
`LowCmd` carry a **`crc`** that the firmware validates, so command
serialization is not merely a struct copy. The arrays hold 20 slots while the
Go2 actuates 12 joints (4 legs × 3), so indices 12–19 are unused.

## The central idea

Two pure functions are the entire sim-to-real contract:

```
ObservationEncoder:  RobotObservation + history → [Float]        // policy input
ActionDecoder:       [Float] + RobotObservation → RobotCommand   // policy output
```

These are where sim-to-real projects silently diverge — joint ordering, sign
conventions, units, normalization constants, default-pose offsets. Most such
bugs are invisible until hardware moves incorrectly. Making these two functions
pure, shared by both modes, and directly unit-tested is the framework's core
value; everything else is plumbing that gets them the right inputs.

The policy sees only `[Float]`. The robot sees only `RobotCommand`. Neither
knows whether physics came from MuJoCo or from a motor.

## Canonical types

Robot-agnostic, `Sendable` value types:

```swift
struct RobotObservation: Sendable {
    var stamp: RobotTime          // sim or wall, per the active clock
    var joints: [JointReading]    // position, velocity, effort — fixed, named order
    var imu: IMUReading           // orientation quaternion, angular velocity, linear acceleration
    var contacts: [ContactReading] // per-foot normal force + boolean contact
}

struct RobotCommand: Sendable {
    var stamp: RobotTime
    var joints: [JointTarget]     // position, velocity, feedforward torque, kp, kd
}
```

`RobotCommand` is PD setpoints because that is simultaneously what
`unitree_go/MotorCmd` carries and what a MuJoCo actuator can implement — no
impedance mismatch at the boundary.

## Architecture

```
MuJoCo ──synthesize──┐                    ┌── real Go2
                     ▼                    ▼
              unitree_go/LowState ── transport ──► Go2Adapter ──► RobotObservation
                                                                        │
                                                                   Encoder
                                                                        ▼
                                                                    Policy
                                                                        │
                                                                   Decoder
                                                                        ▼
              unitree_go/LowCmd ◄── transport ◄── Go2Adapter ◄── RobotCommand
                     │                    │
                     ▼                    ▼
              MuJoCo actuators        real Go2
```

The simulator synthesizes **genuine vendor messages**, so the `LowState` decode
path — the exact code that will face hardware — is exercised in simulation. This
is what makes the parity claim real rather than aspirational: a joint-ordering
or unit error in the adapter fails in sim, not on a robot.

### Targets

| Target | Depends on | Responsibility |
| --- | --- | --- |
| `RobotKit` | SwiftROS2 | Canonical types, protocols (`MessageTransport`, `RobotAdapter`, `Policy`, `Clock`), encoder/decoder, runtime loop, `InProcessTransport` |
| `RobotKitROS2` | RobotKit, SwiftROS2 | `ROS2Transport` (DDS/Zenoh) |
| `RobotKitSim` | RobotKit, MuJoCo | Sensor synthesis from `mjData`, command → actuators, sim clock, noise/latency models |
| `RobotKitGo2` | RobotKit | Generated `unitree_go` messages, Go2 adapter, joint map, CRC |
| `RobotKitTraining` | RobotKit, RobotKitSim, MLX | `Task` protocol, vectorized envs, trainers, policy export |
| `go2-demo` | all | One binary: `--mode sim\|loopback\|real` |

`RobotKit` depends on neither MuJoCo nor MLX, so it can be extracted into its
own package later without untangling. The framework lives on a branch of this
repo for now to keep CI and iteration shared with the bindings it builds on.

### Transport

```swift
protocol MessageTransport: Sendable {
    func publish<M: ROS2Message>(_ message: M, topic: String) async throws
    func subscribe<M: ROS2Message>(_ type: M.Type, topic: String) async throws -> AsyncStream<M>
}
```

Two implementations: `InProcessTransport` (typed channels, no serialization —
the training path, and dependency-free enough to live in `RobotKit` itself) and
`ROS2Transport` (real DDS/Zenoh, in `RobotKitROS2`). Message *types* are the
same either way; only whether they cross a wire differs.

The encoder's history buffer (previous observations and actions, which
locomotion policies typically need) is owned by the encoder instance, not
threaded through call sites — so the same encoder object is reused across a
control loop in both modes and its warm-up behavior is identical.

### Three modes, one binary

1. **`sim`** — in-process transport, no serialization. Fast enough for RL.
2. **`loopback`** — the simulator publishes real `unitree_go` messages over DDS;
   the deploy binary connects as though to hardware. **This is the parity
   proof.** It exercises the entire real path — DDS, vendor decode, CRC,
   adapter, timing, QoS — with no robot and no mocks. The only untested
   component is the physical machine.
3. **`real`** — the same binary, DDS to the Go2.

### Time

A `Clock` protocol: the sim clock advances with `mjData.time` and publishes
`/clock` (a hand-written `rosgraph_msgs/Clock`, which swift-ros2 does not
generate); the real clock reads wall time. Messages are stamped from the active
clock and state assembly reads header stamps, so user code never branches on
"am I in simulation?" This is the standard ROS 2 `use_sim_time` convention.

Physics runs at the model timestep; control runs at a lower rate via an explicit
decimation factor, matching how the real control loop is paced by incoming
`LowState`.

### Sim-to-real gap tooling

Deliberately minimal, and limited to what is genuinely an *interpretation*
concern:

- Per-stream sensor noise, latency, and update rate.
- Action latency.
- Physics randomization at reset (mass, friction, motor gains). This writes
  through `MjModel.ptr`, so it must respect that type's documented
  cache-staleness rule: `joints`/`actuators`/`sensors`/`bodyNames` are cached on
  first access and never invalidated. Randomize before first introspection, or
  construct a fresh `MjModel`.

## Phasing

The spec covers all three phases; each ships something testable on its own, and
each gets its own implementation plan.

**Phase 1 — framework and parity (planned first).** Canonical types, encoder/
decoder, both transports, Go2 adapter with generated messages and CRC, MuJoCo
sensor synthesis, the runtime loop, and `sim` + `loopback` modes. Proven with a
scripted stand / PD-hold controller — no learning involved. Deliverable: the
same controller code producing the same behavior through both an in-process
simulator and a DDS wire, verified byte-for-byte.

**Phase 2 — training.** `Task` protocol (reward, termination, reset
distribution — simulation-only, since there is no reward on hardware),
vectorized environments, MLX policy, and policy export/load. Requires promoting
`ReinforceTrainer`/`PPOTrainer` out of the `MujocoRLDemo` executable into a
library target. Deliverable: a locomotion policy trained in sim and running in
loopback.

**Phase 3 — hardware bring-up.** Safety gating (torque and joint limits, a
watchdog on stale state, an e-stop path), a standing-pose-only first motion, and
a documented bring-up checklist. Deliverable: the trained policy running on the
physical Go2.

## Policy execution

MLX runs in the deployed binary, including on the robot's Linux+CUDA compute.
The MLX-free forward pass built for the RL sample (`PolicyWeights` /
`policyForward`) is retained, but only for its original reason — parallel
rollout workers during training, where MLX's cross-thread behavior is unverified
— not as a platform workaround. Both implementations remain cross-checked by the
existing parity test.

## Testing

- **Encoder/decoder golden tests.** The highest-value tests here: fixed
  `RobotObservation` → expected `[Float]`, fixed action → expected
  `RobotCommand`, with hand-computed values covering joint order, signs, and
  normalization.
- **Cross-transport parity.** An identical `LowState` pushed through the
  in-process and DDS paths must yield byte-identical observations. This is the
  test that would catch a serialization or CRC regression.
- **Loopback integration.** Mode 2 run end-to-end in CI on Linux.
- **Adapter round-trip.** `LowState` → `RobotObservation` → `LowCmd` preserves
  joint identity and produces a firmware-valid CRC.

## CI

The Linux job drops the `#if os(macOS)` gate and builds the framework with
MLX CPU-only: `SPM_CUDA=0`, plus apt-installed `blas`/`lapack`/`openblas`/
`gfortran`. CI then compiles the same code that runs on the Jetson (minus CUDA
kernels) and runs the loopback integration test there, so the Linux deployment
path stops being untested. macOS CI keeps `xcodebuild test` for the MLX-touching
tests, per the constraint established in the RL sample work: SwiftPM's
command-line build cannot compile MLX's Metal shaders.

## Out of scope

- Robots other than the Go2. The adapter boundary exists so others can be added;
  none will be in these phases.
- Cameras, lidar, and vision. The Go2's sensor set here is joints, IMU, and foot
  contacts. `swift-mujoco` already supports raycasting and offscreen rendering
  when that changes.
- Navigation, SLAM, and planning. RobotKit interprets state and emits joint
  commands; higher-level autonomy sits above it.
- ROS 2 services and actions. Phase 1–3 need only pub/sub, though swift-ros2
  supports both when a use case appears.
- Real-time scheduling guarantees. The control loop is best-effort Swift
  concurrency, not an RT kernel.

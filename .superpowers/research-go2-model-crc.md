# Research: MuJoCo Menagerie Go2 model + Unitree LowCmd CRC

Sources:
- `github.com/google-deepmind/mujoco_menagerie`, directory `unitree_go2/` (fetched via `gh api repos/google-deepmind/mujoco_menagerie/contents/unitree_go2/...`)
- `github.com/unitreerobotics/unitree_sdk2` (C++ SDK)
- `github.com/unitreerobotics/unitree_ros2` (ROS 2 example code)
- `github.com/unitreerobotics/unitree_sdk2_python` (Python SDK)

All code blocks below are quoted verbatim from the fetched files.

---

## Part 1: MuJoCo Menagerie Go2 model

### 1. Files present in `unitree_go2/`

```
CHANGELOG.md
LICENSE
README.md
assets
go2.png
go2.xml
go2_mjx.png
go2_mjx.xml
scene.xml
scene_mjx.xml
```

Two model variants exist, and **they are meaningfully different for a sim-to-real framework**:

- **`go2.xml` / `scene.xml`** — the "physical" model. Actuators are pure `<motor>` (raw torque) actuators. **No `<sensor>` block at all.** This is the correct base for a framework that must implement Unitree's own variable-gain `MotorCmd(q, dq, tau, kp, kd)` PD semantics, because it leaves PD control entirely up to the framework (matching the real robot, where kp/kd are sent per command and can vary).
- **`go2_mjx.xml` / `scene_mjx.xml`** — an MJX/RL-training-oriented variant. Actuators are `<general>` actuators with `biastype="affine"`, `gainprm="50 0 0"`, `biasprm="0 -50 -0.5"` — i.e. **a PD controller with fixed, baked-in gains (kp=50, kd=0.5, forcerange ±24)** compiled into the model itself. `ctrl` here is a desired joint angle, not a torque. It also has a full `<sensor>` block and extra foot `<site>`s. Because the gains are fixed at compile time, this variant **cannot** reproduce Unitree's per-command variable kp/kd — if the framework needs to send different kp/kd per LowCmd (as real hardware does), it should use `go2.xml`'s torque-motor actuators and implement the PD loop itself in Swift, not the MJX variant.

`scene.xml` only `<include file="go2.xml"/>` — the default/primary scene uses the torque-motor model with **no sensors declared**.

### 2. Joint names, order, types, ranges (from `go2.xml`)

The kinematic tree order (as bodies appear under `worldbody`) is **FL → FR → RL → RR**, and within each leg **hip → thigh → calf**:

| # | Joint name | Type | Range (rad) | Class |
|---|---|---|---|---|
| — | (unnamed) `<freejoint/>` on body `base` | free | n/a | — |
| 1 | `FL_hip_joint` | hinge | `-1.0472 1.0472` | abduction |
| 2 | `FL_thigh_joint` | hinge | `-1.5708 3.4907` | front_hip |
| 3 | `FL_calf_joint` | hinge | `-2.7227 -0.83776` | knee |
| 4 | `FR_hip_joint` | hinge | `-1.0472 1.0472` | abduction |
| 5 | `FR_thigh_joint` | hinge | `-1.5708 3.4907` | front_hip |
| 6 | `FR_calf_joint` | hinge | `-2.7227 -0.83776` | knee |
| 7 | `RL_hip_joint` | hinge | `-1.0472 1.0472` | abduction |
| 8 | `RL_thigh_joint` | hinge | `-0.5236 4.5379` | back_hip |
| 9 | `RL_calf_joint` | hinge | `-2.7227 -0.83776` | knee |
| 10 | `RR_hip_joint` | hinge | `-1.0472 1.0472` | abduction |
| 11 | `RR_thigh_joint` | hinge | `-0.5236 4.5379` | back_hip |
| 12 | `RR_calf_joint` | hinge | `-2.7227 -0.83776` | knee |

**qpos layout**: `qpos[0:3]` = free-joint position (x,y,z), `qpos[3:7]` = free-joint quaternion (w,x,y,z), `qpos[7:19]` = the 12 joint angles above, in that exact order (FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf). `qvel` is analogous but 6+12=18-dim (free joint velocity is 6-dim: linvel+angvel).

**IMPORTANT gotcha — leg ordering mismatch with real firmware**: the Menagerie model orders legs **FL, FR, RL, RR**. Unitree's real `MotorCmd`/`LowState` arrays use a **different** canonical order — from `unitree_ros2`'s `motor_crc.h`:

```cpp
// joint index
constexpr int FR_0 = 0;
constexpr int FR_1 = 1;
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;
```

i.e. real firmware order is **FR, FL, RR, RL** (motor index 0-11), while the MuJoCo model's joint/actuator order is **FL, FR, RL, RR**. Any sim-to-real mapping layer MUST remap between these two orderings explicitly — this is exactly the kind of silent bug the plan needs to guard against.

Per-joint dynamics defaults (from the `go2` default class, apply to all 12 joints unless overridden — none override these): `damping="2"`, `armature="0.01"`, `frictionloss="0.2"`. Default joint `axis="0 1 0"`, overridden to `axis="1 0 0"` for abduction-class (hip) joints.

### 3. Actuators — order, type, ctrlrange, gear/kp/kv

From `go2.xml`'s `<actuator>` block, verbatim:

```xml
<actuator>
  <motor class="abduction" name="FL_hip" joint="FL_hip_joint"/>
  <motor class="hip" name="FL_thigh" joint="FL_thigh_joint"/>
  <motor class="knee" name="FL_calf" joint="FL_calf_joint"/>
  <motor class="abduction" name="FR_hip" joint="FR_hip_joint"/>
  <motor class="hip" name="FR_thigh" joint="FR_thigh_joint"/>
  <motor class="knee" name="FR_calf" joint="FR_calf_joint"/>
  <motor class="abduction" name="RL_hip" joint="RL_hip_joint"/>
  <motor class="hip" name="RL_thigh" joint="RL_thigh_joint"/>
  <motor class="knee" name="RL_calf" joint="RL_calf_joint"/>
  <motor class="abduction" name="RR_hip" joint="RR_hip_joint"/>
  <motor class="hip" name="RR_thigh" joint="RR_thigh_joint"/>
  <motor class="knee" name="RR_calf" joint="RR_calf_joint"/>
</actuator>
```

All 12 are **`<motor>` type = pure torque actuators** (`ctrl` maps directly to joint torque via `gear=1` default; no `gainprm`/`biasprm`/`kp`/`kv`). This confirms: **the standard Go2 Menagerie model does NOT implement PD control internally** — `ctrl[i]` is a torque in N·m, full stop. The framework must implement its own PD loop (`tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff`) to match Unitree's `MotorCmd(q, dq, tau, kp, kd)` semantics and feed the resulting torque into `ctrl`.

`ctrlrange` values (via the default hierarchy):
```xml
<default class="go2">
  ...
  <motor ctrlrange="-23.7 23.7"/>
  ...
  <default class="knee">
    <joint range="-2.7227 -0.83776"/>
    <motor ctrlrange="-45.43 45.43"/>
  </default>
  ...
</default>
```
- `abduction` (hip roll) actuators — `FL_hip`, `FR_hip`, `RL_hip`, `RR_hip` — inherit `ctrlrange="-23.7 23.7"` (N·m) from the `go2` default (no override in `abduction`).
- `hip` (thigh/pitch) actuators — `FL_thigh`, `FR_thigh`, `RL_thigh`, `RR_thigh` — also inherit `ctrlrange="-23.7 23.7"` (the `hip`/`front_hip`/`back_hip` classes only override joint `range`, not the motor).
- `knee` (calf) actuators — `FL_calf`, `FR_calf`, `RL_calf`, `RR_calf` — override to `ctrlrange="-45.43 45.43"`.

No `gear`, `kp`, `kv`, or `forcerange` attributes are set on any actuator (gear defaults to 1; there is no separate forcerange beyond ctrlrange since these are motor actuators with `ctrllimited` implied by `ctrlrange` via `autolimits="true"` in `<compiler>`).

(Contrast: `go2_mjx.xml`'s `<general>` actuators use `biastype="affine" gainprm="50 0 0" biasprm="0 -50 -0.5"` with `forcerange="-24 24"`, i.e. fixed kp=50, kd=0.5 baked into the model — not usable for per-command variable-gain PD.)

### 4. Sensors

**`go2.xml` (the standard model used by `scene.xml`) declares NO `<sensor>` block at all.** There is only a `<site name="imu" pos="-0.02557 0 0.04232"/>` on the `base` body — a site exists to attach sensors to, but nothing reads it. **All IMU/proprioceptive data (orientation, gyro, accel, joint pos/vel) must be computed/read directly from MuJoCo state (`qpos`/`qvel`/body transforms) by the framework**, not pulled from `data.sensordata`, unless the framework adds its own `<sensor>` elements to the model.

For reference, the **MJX variant** (`go2_mjx.xml`) does declare a full sensor suite (useful as a template for what to add):
```xml
<sensor>
  <jointpos joint="FL_hip_joint" name="abduction_front_left_pos"/>
  <jointpos joint="FL_thigh_joint" name="hip_front_left_pos"/>
  <jointpos joint="FL_calf_joint" name="knee_front_left_pos"/>
  <jointpos joint="RL_hip_joint" name="abduction_hind_left_pos"/>
  <jointpos joint="RL_thigh_joint" name="hip_hind_left_pos"/>
  <jointpos joint="RL_calf_joint" name="knee_hind_left_pos"/>
  <jointpos joint="FR_hip_joint" name="abduction_front_right_pos"/>
  <jointpos joint="FR_thigh_joint" name="hip_front_right_pos"/>
  <jointpos joint="FR_calf_joint" name="knee_front_right_pos"/>
  <jointpos joint="RR_hip_joint" name="abduction_hind_right_pos"/>
  <jointpos joint="RR_thigh_joint" name="hip_hind_right_pos"/>
  <jointpos joint="RR_calf_joint" name="knee_hind_right_pos"/>
  <jointvel joint="FL_hip_joint" name="abduction_front_left_vel"/>
  <jointvel joint="FL_thigh_joint" name="hip_front_left_vel"/>
  <jointvel joint="FL_calf_joint" name="knee_front_left_vel"/>
  <jointvel joint="RL_hip_joint" name="abduction_hind_left_vel"/>
  <jointvel joint="RL_thigh_joint" name="hip_hind_left_vel"/>
  <jointvel joint="RL_calf_joint" name="knee_hind_left_vel"/>
  <jointvel joint="FR_hip_joint" name="abduction_front_right_vel"/>
  <jointvel joint="FR_thigh_joint" name="hip_front_right_vel"/>
  <jointvel joint="FR_calf_joint" name="knee_front_right_vel"/>
  <jointvel joint="RR_hip_joint" name="abduction_hind_right_vel"/>
  <jointvel joint="RR_thigh_joint" name="hip_hind_right_vel"/>
  <jointvel joint="RR_calf_joint" name="knee_hind_right_vel"/>
  <gyro site="imu" name="gyro"/>
  <accelerometer site="imu" name="accelerometer"/>
  <framequat objtype="site" objname="imu" name="orientation"/>
  <framepos objtype="site" objname="imu" name="global_position"/>
  <framelinvel objtype="site" objname="imu" name="global_linvel"/>
  <frameangvel objtype="site" objname="imu" name="global_angvel"/>
</sensor>
```
Note even here, "orientation" is `framequat` (ground-truth, not filtered IMU-style), `global_position`/`global_linvel`/`global_angvel` are ground-truth world-frame quantities not available on real hardware — only `gyro` and `accelerometer` at the `imu` site are physically realistic sensor analogs.

### 5. Feet / trunk body & site names

- **Trunk/base body**: `base` (contains the `<freejoint/>`, the IMU site, and the main collision geoms).
- **IMU site**: `<site name="imu" pos="-0.02557 0 0.04232"/>` on body `base`.
- **Feet** (in `go2.xml`, the standard model): feet are represented as **geoms**, not sites — `<geom name="FL" class="foot"/>`, `<geom name="RR" class="foot"/>`, etc., one per calf body (`FL_calf`, `FR_calf`, `RL_calf`, `RR_calf`). Class `foot` definition: `<geom size="0.022" pos="-0.002 0 -0.213" priority="1" solimp="0.015 1 0.022" condim="6" friction="0.8 0.02 0.01"/>`. These are the geoms to check in `contact` pairs for foot-contact detection (match by `geom1`/`geom2` name `FL`/`FR`/`RL`/`RR`).
- The **MJX variant** additionally adds foot `<site>`s (`FL_foot`, `FR_foot`, `RL_foot`, `RR_foot`, class `go2foot`) at the same offset — useful if the framework wants named sites for foot position/velocity rather than relying on the collision geom.

### 6. Keyframe ("home" stand pose)

From `go2.xml`:
```xml
<keyframe>
  <key name="home" qpos="0 0 0.27 1 0 0 0 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"
    ctrl="0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"/>
</keyframe>
```
(Identical in `go2_mjx.xml`.)

Decoded `qpos` (19 values = 7 free-joint + 12 joint angles, in the joint order from §2):
- Base position: `x=0, y=0, z=0.27` (standing height, meters)
- Base quaternion: `w=1, x=0, y=0, z=0` (upright, identity)
- Joint angles (rad), per leg, hip/thigh/calf, in FL,FR,RL,RR order: `hip=0, thigh=0.9, calf=-1.8` for all four legs.

**Gotcha**: the `ctrl="0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"` values numerically equal the joint-angle values, but since these actuators are pure torque `<motor>`s, `ctrl` is interpreted as **torque in N·m**, not a target angle. Applying this `ctrl` vector directly will NOT hold the stand pose — it looks like the keyframe author reused the qpos values as a placeholder ctrl array (a common Menagerie convention) rather than a physically meaningful torque command. **A real PD controller** (using `qpos`'s joint angles as `q_des` with 0 `dq_des`, wrapped through the framework's own kp/kd, matching Unitree's `MotorCmd`) is required to actually hold this stand pose in sim.

### 7. `<option>` settings

`go2.xml`:
```xml
<option cone="elliptic" impratio="100"/>
```
**No `timestep` attribute is set** — this means MuJoCo's default `timestep="0.002"` (i.e. 500 Hz / 2 ms) applies. `scene.xml` adds no `<option>` override either.

For contrast, `go2_mjx.xml` (not used by the default `scene.xml`) sets:
```xml
<option cone="pyramidal" impratio="100" iterations="1" ls_iterations="5">
  <flag eulerdamp="disable"/>
</option>
```
also with no explicit `timestep` (defaults to 0.002s), but different contact cone/solver settings tuned for MJX throughput, not accuracy — not representative of what the standard `go2.xml`/`scene.xml` uses.

---

## Part 2: Unitree LowCmd CRC

### Verdict

This is **Unitree's own custom CRC-32 variant**, NOT the standard CRC-32 (IEEE 802.3 / zlib) polynomial-reflected algorithm. It is a **bit-by-bit, MSB-first, non-reflected** CRC-32 using polynomial `0x04C11DB7`, initial value `0xFFFFFFFF`, computed by iterating **32-bit words** (not bytes) taken directly from the raw in-memory struct layout, and it excludes the trailing `crc` field itself from the computation. It was found consistently and independently in three separate Unitree repos (C++ SDK, ROS2 example, Python SDK), so this is verified, not guessed.

### 2a. The exact algorithm — verbatim reference source

From `unitree_sdk2/include/unitree/dds_wrapper/common/crc.h`:
```c
inline uint32_t crc32_core(uint32_t* ptr, uint32_t len){
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}
```

Identical (modulo whitespace/brace style) in `unitree_ros2/example/src/src/common/motor_crc.cpp`:
```cpp
uint32_t crc32_core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t CRC32 = 0xFFFFFFFF;
  const uint32_t dwPolynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1 << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {
        CRC32 <<= 1;
        CRC32 ^= dwPolynomial;
      } else
        CRC32 <<= 1;
      if (data & xbit) CRC32 ^= dwPolynomial;

      xbit >>= 1;
    }
  }
  return CRC32;
}
```

And a pure-Python re-implementation in `unitree_sdk2_python/unitree_sdk2py/utils/crc.py` (used as the non-Linux fallback, i.e. this is Unitree's own confirmation of the bit-level semantics independent of C aliasing/endianness questions):
```python
def _crc_py(self, data):
    bit = 0
    crc = 0xFFFFFFFF
    polynomial = 0x04c11db7

    for i in range(len(data)):
        bit = 1 << 31
        current = data[i]

        for b in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) & 0xFFFFFFFF
                crc ^= polynomial
            else:
                crc = (crc << 1) & 0xFFFFFFFF

            if current & bit:
                crc ^= polynomial

            bit >>= 1

    return crc
```
(`data` here is a list of 32-bit words already reassembled from little-endian bytes — see §2c.)

### 2b. What buffer it's computed over — the word-count formula

From `unitree_sdk2/example/go2/go2_low_level.cpp` and `unitree_sdk2/example/state_machine/robot_interface.hpp`:
```cpp
low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
```
and equivalently in `unitree_sdk2/include/unitree/dds_wrapper/robots/go2/go2_pub.h`:
```cpp
msg_.crc() = crc32_core((uint32_t*)&msg_, (sizeof(MsgType)>>2)-1);
```

This reads the **entire packed `LowCmd_` struct as an array of 32-bit little-endian words**, and passes `len = (sizeof(LowCmd_) / 4) - 1` — i.e. **every word of the struct except the very last one, which is the `crc` field itself**. The result is written back into that last word (`crc`).

### 2c. Exact byte layout of `LowCmd_` (confirms word alignment/padding)

From `unitree_ros2/example/src/include/common/motor_crc.h` (the "raw" plain-C struct used for CRC purposes on Go2/B2, matching field-for-field the DDS IDL-generated `unitree_go::msg::dds_::LowCmd_`):
```c
typedef struct {
  uint8_t off;  // off 0xA5
  std::array<uint8_t, 3> reserve;
} BmsCmd;

typedef struct {
  uint8_t mode;  // desired working mode
  float q;       // desired angle (unit: radian)
  float dq;      // desired velocity (unit: radian/second)
  float tau;     // desired output torque (unit: N.m)
  float Kp;      // desired position stiffness (unit: N.m/rad )
  float Kd;      // desired velocity stiffness (unit: N.m/(rad/s) )
  std::array<uint32_t, 3> reserve;
} MotorCmd;  // motor control

typedef struct {
  std::array<uint8_t, 2> head;
  uint8_t levelFlag;
  uint8_t frameReserve;

  std::array<uint32_t, 2> SN;
  std::array<uint32_t, 2> version;
  uint16_t bandWidth;
  std::array<MotorCmd, 20> motorCmd;
  BmsCmd bms;
  std::array<uint8_t, 40> wirelessRemote;
  std::array<uint8_t, 12> led;
  std::array<uint8_t, 2> fan;
  uint8_t gpio;
  uint32_t reserve;

  uint32_t crc;
} LowCmd;
```

The DDS-generated C++ class (`unitree_sdk2/include/unitree/idl/go2/LowCmd_.hpp`) has the identical field set/order:
```cpp
class LowCmd_
{
private:
 std::array<uint8_t, 2> head_ = { };
 uint8_t level_flag_ = 0;
 uint8_t frame_reserve_ = 0;
 std::array<uint32_t, 2> sn_ = { };
 std::array<uint32_t, 2> version_ = { };
 uint16_t bandwidth_ = 0;
 std::array<::unitree_go::msg::dds_::MotorCmd_, 20> motor_cmd_ = { };
 ::unitree_go::msg::dds_::BmsCmd_ bms_cmd_;
 std::array<uint8_t, 40> wireless_remote_ = { };
 std::array<uint8_t, 12> led_ = { };
 std::array<uint8_t, 2> fan_ = { };
 uint8_t gpio_ = 0;
 uint32_t reserve_ = 0;
 uint32_t crc_ = 0;
```
with `MotorCmd_`:
```cpp
class MotorCmd_
{
private:
 uint8_t mode_ = 0;
 float q_ = 0.0f;
 float dq_ = 0.0f;
 float tau_ = 0.0f;
 float kp_ = 0.0f;
 float kd_ = 0.0f;
 std::array<uint32_t, 3> reserve_ = { };
```
and `BmsCmd_`:
```cpp
class BmsCmd_
{
private:
 uint8_t off_ = 0;
 std::array<uint8_t, 3> reserve_ = { };
```

**Total struct size is 812 bytes = 203 × 4-byte words**, confirmed independently by the Python SDK's packed-struct comment in `unitree_sdk2py/utils/crc.py`:
```python
#4 bytes aligned, little-endian format.
#size 812
self.__packFmtLowCmd = '<4B4IH2x' + 'B3x5f3I' * 20 + '4B' + '55Bx2I'
```
So `crc32_core` is called with `len = (812 >> 2) - 1 = 202` words = the first **808 bytes** of the struct; the last word (bytes 808-811) is the `crc` field, written after the call.

Byte-offset walk (little-endian, natural C struct alignment, 4-byte struct alignment due to `float`/`uint32_t` members):
| Field | Offset | Size |
|---|---|---|
| `head[2]` | 0 | 2 |
| `level_flag` | 2 | 1 |
| `frame_reserve` | 3 | 1 |
| `sn[2]` (uint32×2) | 4 | 8 |
| `version[2]` (uint32×2) | 12 | 8 |
| `bandwidth` (uint16) | 20 | 2 |
| *(2 bytes padding to 4-byte align `motor_cmd`)* | 22 | 2 |
| `motor_cmd[20]` (each 36 B: `mode`(1)+pad(3)+`q,dq,tau,kp,kd`(20)+`reserve[3]`(12)) | 24 | 720 |
| `bms_cmd` (`off`(1)+`reserve[3]`(3)) | 744 | 4 |
| `wireless_remote[40]` | 748 | 40 |
| `led[12]` | 788 | 12 |
| `fan[2]` | 800 | 2 |
| `gpio` | 802 | 1 |
| *(1 byte padding to 4-byte align `reserve`)* | 803 | 1 |
| `reserve` (uint32) | 804 | 4 |
| `crc` (uint32) | 808 | 4 |
| **total** | | **812** |

This matches the Python `struct.pack` format exactly (`'<4B4IH2x'` = 4+16+2+2=24 bytes; `'B3x5f3I'×20` = 36×20=720 bytes; `'4B'` = 4 bytes; `'55Bx2I'` = 55+1+8=64 bytes; 24+720+4+64=812) — cross-validated from an entirely independent source (Python re-serialization vs. C++ struct layout).

### 2d. Gotchas for a Swift reimplementation

1. **Not a byte-wise CRC — it's word-wise.** The algorithm processes `uint32_t` words directly out of memory (`data = ptr[i]`), not bytes. On a little-endian host (x86/ARM, which the real robot's onboard computer and virtually all dev machines are), reading a `uint32_t` from a byte buffer means **bytes `[4i], [4i+1], [4i+2], [4i+3]` combine as `word = b0 | (b1<<8) | (b2<<16) | (b3<<24)`** — exactly what the Python SDK's `__Trans` method does explicitly for portability:
   ```python
   def __Trans(self, packData):
       calcData = []
       calcLen = ((len(packData)>>2)-1)
       for i in range(calcLen):
           d = ((packData[i*4+3] << 24) | (packData[i*4+2] << 16) | (packData[i*4+1] << 8) | (packData[i*4]))
           calcData.append(d)
       return calcData
   ```
   A Swift implementation should build the word array the same way (little-endian byte→word packing) rather than relying on `withUnsafeBytes` word reinterpretation if there's any doubt about the host's endianness — but since both the Go2's onboard compute and any Mac/Linux dev machine are little-endian, direct pointer reinterpretation is also safe in practice.

2. **Struct padding is load-bearing.** The struct has two implicit padding regions (2 bytes after `bandwidth`, 1 byte after `gpio`) required for 4-byte alignment of the following `uint32_t`/`float` fields. **These padding bytes must be zeroed and included in the CRC input** — omitting them or packing the struct tightly (e.g., `#pragma pack(1)`) will shift every subsequent field and produce a completely different (and wrong) CRC. A Swift port should serialize the LowCmd fields into a flat byte buffer using the *exact* offsets/padding above (or replicate the struct with the same alignment, e.g. as a manually laid-out byte buffer) rather than relying on Swift's own struct layout rules (which are not guaranteed to match C's).

3. **CRC excludes only the last word (the crc field), not "everything before some header field".** It's computed over `head, levelFlag, frameReserve, SN, version, bandWidth, motorCmd[20], bms, wirelessRemote, led, fan, gpio, reserve` — i.e., the *entire* message payload including all 20 motor slots (even though Go2 only uses 12) — and excludes only the trailing `crc` word itself.

4. **Initial value is `0xFFFFFFFF`, not `0x00000000`**, and there is **no final XOR** (the raw computed `CRC32` register value is used directly as-is) — unlike standard CRC-32 (which XORs the final result with `0xFFFFFFFF`). This "no-reflect, no-final-XOR, init=0xFFFFFFFF" combination plus `dwPolynomial=0x04C11DB7` is exactly why standard CRC-32 libraries produce different values.

5. **Same function/struct pattern is reused for `LowState_`** (feedback message from the robot) and for the `unitree_hg` (humanoid) message family, just with different struct layouts (`sizeof(LowState_)` etc.) — the CRC function itself (`crc32_core`) is identical everywhere; only the buffer being fed to it changes.

### Files/paths referenced (for follow-up verification)
- `unitree_sdk2/include/unitree/dds_wrapper/common/crc.h`
- `unitree_sdk2/include/unitree/idl/go2/LowCmd_.hpp`, `MotorCmd_.hpp`, `BmsCmd_.hpp`
- `unitree_sdk2/example/go2/go2_low_level.cpp`
- `unitree_sdk2/example/state_machine/robot_interface.hpp`
- `unitree_sdk2/include/unitree/dds_wrapper/robots/go2/go2_pub.h`
- `unitree_ros2/example/src/include/common/motor_crc.h`
- `unitree_ros2/example/src/src/common/motor_crc.cpp`
- `unitree_sdk2_python/unitree_sdk2py/utils/crc.py`
- `mujoco_menagerie/unitree_go2/go2.xml`, `scene.xml`, `go2_mjx.xml`, `scene_mjx.xml`

import Foundation

enum Fixtures {
    /// A single hinge pole with one actuator — for joints/actuators/data tests.
    static let pendulum = """
    <mujoco>
      <worldbody>
        <body name="pole" pos="0 0 1">
          <joint name="hinge" type="hinge" axis="0 1 0" range="-179.9 179.9" limited="true"/>
          <geom name="rod" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02" rgba="0.8 0.2 0.2 1"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="mot" joint="hinge" ctrlrange="-1 1" ctrllimited="true"/>
      </actuator>
    </mujoco>
    """

    /// Floor + a free-floating cube with one visible and one hidden (group 3) geom.
    static let boxScene = """
    <mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
        <body name="cube" pos="0 0 1">
          <freejoint/>
          <geom name="box" type="box" size="0.1 0.1 0.1" rgba="0.1 0.5 0.9 1" group="0"/>
          <geom name="hidden" type="box" size="0.1 0.1 0.1" group="3"/>
        </body>
      </worldbody>
    </mujoco>
    """

    /// A geom whose color comes from a `<material>` asset, not its own rgba —
    /// used to test the `geom_matid >= 0 -> mat_rgba` resolution branch.
    static let materialScene = """
    <mujoco>
      <asset>
        <material name="red" rgba="0.9 0.1 0.1 1"/>
      </asset>
      <worldbody>
        <geom name="m" type="box" size="0.1 0.1 0.1" material="red"/>
      </worldbody>
    </mujoco>
    """

    /// A mesh from inline vertices (MuJoCo builds the convex hull faces).
    static let meshScene = """
    <mujoco>
      <asset>
        <mesh name="tri" vertex="0 0 0  1 0 0  0 1 0  0 0 1"/>
      </asset>
      <worldbody>
        <geom type="mesh" mesh="tri" rgba="1 1 1 1"/>
      </worldbody>
    </mujoco>
    """

    /// A free body carrying the five sensor kinds a robot plant reads, held
    /// motionless at its initial pose by `gravcomp="1"` (an exact, non-iterative
    /// force term — not a compliant constraint), so qvel == qacc == 0 with no
    /// settling required.
    ///
    /// Deviation from the original task-1 brief: the brief's fixture made
    /// `probe` jointless (no `<freejoint/>`). MuJoCo's `mj_objectAcceleration`
    /// short-circuits to an all-zero reading for any object whose body is
    /// welded all the way to the world (`body_weldid == 0`, true for any body
    /// with no joint at all), regardless of gravity — so the accelerometer and
    /// gyro read exactly 0, not -gravity, on such a body. That short-circuit is
    /// intentional MuJoCo behavior (see `mj_objectAcceleration` in
    /// `engine_core_util.c`), not a bug in this binding. Giving `probe` a
    /// `<freejoint/>` moves it out of the world's weld group, and
    /// `gravcomp="1"` keeps it stationary without introducing a compliant
    /// constraint (whose finite stiffness would leave the accelerometer a few
    /// percent short of 9.81 and fail the test's tolerance). A `<weld>`
    /// equality constraint was tried first and rejected for exactly that
    /// reason: it settles to ~8.83 m/s^2 with MuJoCo's default solref/solimp,
    /// not 9.81.
    static let sensorScene = """
    <mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="5 5 0.1"/>
        <body name="probe" pos="0 0 0.5" gravcomp="1">
          <freejoint/>
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
}

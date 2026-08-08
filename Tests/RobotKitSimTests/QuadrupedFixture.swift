// Tests/RobotKitSimTests/QuadrupedFixture.swift

enum QuadrupedFixture {
    /// A 12-joint quadruped with Menagerie's joint names, declaration order
    /// (FL, FR, RL, RR), torque actuators, an `imu` site with sensors, and
    /// four named foot geoms — the same surface `Go2Simulator` reads.
    static let xml = """
        <mujoco model="fixture_quadruped">
          <compiler angle="radian"/>
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
              <!-- Upper bound is 0.1, not the real Go2's -0.83: the "stand"
              command tests below drive every joint toward 0, and if the
              calf's range excludes 0 the PD spends the whole run fighting
              the limit, which (verified empirically) makes all four legs
              buckle and the trunk settle flat on the floor by t≈1.5s. This
              fixture only needs to exercise joint-name mapping faithfully
              (see the type doc above) — not reproduce the real robot's
              mechanical stop — so the range is widened just enough to make
              "all joints at 0" a reachable, stable stance. -->
              <joint name="\(p)_calf_joint" type="hinge" axis="0 1 0" range="-2.72 0.1"/>
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

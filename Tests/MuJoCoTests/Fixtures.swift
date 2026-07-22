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
}

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

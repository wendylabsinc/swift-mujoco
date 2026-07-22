import Testing
@testable import MuJoCo

@Test func vec3Ops() {
    let a = Vec3(1, 0, 0), b = Vec3(0, 1, 0)
    #expect(a.cross(b).array == [0, 0, 1])
    #expect(a.dot(b) == 0)
    #expect(abs(Vec3(3, 4, 0).norm - 5) < 1e-12)
}

@Test func identityMatrixIsIdentityQuat() {
    let q = mat2Quat([1,0,0, 0,1,0, 0,0,1])
    #expect(abs(q.w - 1) < 1e-9)
    #expect(abs(q.x) < 1e-9 && abs(q.y) < 1e-9 && abs(q.z) < 1e-9)
}

@Test func geomPoseReadable() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    mjForward(m, d)
    let p = d.geomXpos(1)          // the cube geom, declared at body pos z=1
    #expect(abs(p.z - 1) < 1e-6)
    #expect(d.geomXmat(1).count == 9)
}

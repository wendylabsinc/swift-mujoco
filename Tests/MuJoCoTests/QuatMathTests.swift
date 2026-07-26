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

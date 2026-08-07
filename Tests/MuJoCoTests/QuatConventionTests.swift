import Testing
import CMuJoCo
@testable import MuJoCo

// These tests pin the Swift quaternion helpers to MuJoCo's *own* implementations
// by calling mju_* directly. Before the review fixes, `subQuat` computed
// `a ⊗ conj(b)` while carrying the name of `mju_subQuat`, which computes
// `conj(b) ⊗ a` — a body-frame vs world-frame difference that silently rotated
// any orientation error fed to a controller. `negQuat` had the mirror problem:
// it negated all four components, while C's `mju_negQuat` conjugates.

private func mjuSubQuat(_ a: Quat, _ b: Quat) -> Vec3 {
    var res = [Double](repeating: 0, count: 3)
    var qa = [a.w, a.x, a.y, a.z]
    var qb = [b.w, b.x, b.y, b.z]
    mju_subQuat(&res, &qa, &qb)
    return Vec3(res[0], res[1], res[2])
}

private func mjuNegQuat(_ q: Quat) -> Quat {
    var res = [Double](repeating: 0, count: 4)
    var qq = [q.w, q.x, q.y, q.z]
    mju_negQuat(&res, &qq)
    return Quat(w: res[0], x: res[1], y: res[2], z: res[3])
}

private func mjuRotVecQuat(_ v: Vec3, _ q: Quat) -> Vec3 {
    var res = [Double](repeating: 0, count: 3)
    var vv = [v.x, v.y, v.z]
    var qq = [q.w, q.x, q.y, q.z]
    mju_rotVecQuat(&res, &vv, &qq)
    return Vec3(res[0], res[1], res[2])
}

private func expectClose(_ a: Vec3, _ b: Vec3, _ tol: Double = 1e-12,
                         _ label: String = "", sourceLocation: SourceLocation = #_sourceLocation) {
    #expect(abs(a.x - b.x) < tol && abs(a.y - b.y) < tol && abs(a.z - b.z) < tol,
            "\(label) \(a.array) != \(b.array)", sourceLocation: sourceLocation)
}

/// A spread of orientation pairs, including near-identity and near-π cases.
private let quatPairs: [(Quat, Quat)] = {
    let samples = [
        Quat.identity,
        euler2Quat(roll: 0.3, pitch: -0.2, yaw: 1.1),
        euler2Quat(roll: -1.4, pitch: 0.9, yaw: -2.2),
        euler2Quat(roll: 0.0, pitch: 0.0, yaw: 3.10),      // close to pi
        euler2Quat(roll: 1e-9, pitch: 0, yaw: 0),          // close to identity
        euler2Quat(roll: 2.9, pitch: -1.2, yaw: 0.4),
    ]
    var out: [(Quat, Quat)] = []
    for a in samples { for b in samples { out.append((a, b)) } }
    return out
}()

@Test func subQuatMatchesMjuSubQuat() {
    for (a, b) in quatPairs {
        expectClose(subQuat(a, b), mjuSubQuat(a, b), 1e-9, "subQuat vs mju_subQuat")
    }
}

@Test func subQuatIsBodyFrameNotWorldFrame() {
    // The two conventions agree only when the axis happens to be invariant under
    // the rotation (e.g. coaxial rotations). Pick a pair where they must differ,
    // so a regression that swaps the multiplication order back cannot pass.
    let a = euler2Quat(roll: 0, pitch: 0, yaw: 1.2)
    let b = euler2Quat(roll: 1.1, pitch: 0, yaw: 0)
    let body = subQuat(a, b)
    let world = subQuatWorld(a, b)
    #expect(abs(body.norm - world.norm) < 1e-12, "same angle, different frame")
    #expect((body - world).norm > 1e-3, "body- and world-frame errors must differ here")
    // And the world-frame variant is exactly the old behaviour: a ⊗ conj(b).
    expectClose(world, quatToRotationVector(mulQuat(a, invQuat(b))), 1e-12, "subQuatWorld")
}

@Test func subQuatOfEqualOrientationsIsZero() {
    let q = euler2Quat(roll: 0.4, pitch: 0.5, yaw: -0.6)
    #expect(subQuat(q, q).norm < 1e-12)
    #expect(subQuatWorld(q, q).norm < 1e-12)
    // Sign-flipped quaternions are the same rotation.
    #expect(subQuat(q, flipQuatSign(q)).norm < 1e-12)
}

@Test func conjQuatMatchesMjuNegQuat() {
    for (q, _) in quatPairs {
        let mine = conjQuat(q)
        let theirs = mjuNegQuat(q)
        #expect(abs(mine.w - theirs.w) < 1e-15)
        #expect(abs(mine.x - theirs.x) < 1e-15)
        #expect(abs(mine.y - theirs.y) < 1e-15)
        #expect(abs(mine.z - theirs.z) < 1e-15)
    }
    // conjQuat is invQuat under another name, and both differ from flipQuatSign.
    let q = euler2Quat(roll: 0.2, pitch: 0.3, yaw: 0.4)
    #expect(conjQuat(q) == invQuat(q))
    #expect(conjQuat(q) != flipQuatSign(q))
}

@Test func flipQuatSignNegatesAllFourComponents() {
    let q = Quat(w: 0.5, x: -0.5, y: 0.5, z: 0.5)
    let f = flipQuatSign(q)
    #expect(f.w == -0.5 && f.x == 0.5 && f.y == -0.5 && f.z == -0.5)
    // Same rotation, so rotating a vector by either gives the same answer.
    let v = Vec3(0.3, -0.7, 0.2)
    expectClose(rotVecQuat(v, q), rotVecQuat(v, f), 1e-12, "flipQuatSign is same rotation")
}

@Test func rotVecQuatMatchesMjuRotVecQuat() {
    let vectors = [Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1),
                   Vec3(0.3, -0.7, 0.2), Vec3(-5, 12, 0.001)]
    for (q, _) in quatPairs {
        for v in vectors {
            expectClose(rotVecQuat(v, q), mjuRotVecQuat(v, q), 1e-12, "rotVecQuat")
        }
    }
}

@Test func rotVecQuatStillMatchesTheMatrixRoute() {
    // rotVecQuat switched from building a Mat3 to calling mju_rotVecQuat to stop
    // allocating three arrays per call; the answer must not have moved.
    for (q, _) in quatPairs {
        let v = Vec3(0.4, -0.25, 1.5)
        expectClose(rotVecQuat(v, q), quat2Mat(q).times(v), 1e-12, "quat vs matrix route")
    }
}

@Test func quatToRotationVectorMatchesMjuQuat2Vel() {
    for (a, b) in quatPairs {
        // mju_subQuat is quat2Vel(conj(b) ⊗ a, 1), so this exercises the
        // rotation-vector conversion against MuJoCo through that path.
        let d = mulQuat(invQuat(b), a)
        expectClose(quatToRotationVector(d), mjuSubQuat(a, b), 1e-9, "quatToRotationVector")
    }
}

@Test func mat2QuatBufferOverloadMatchesArrayOverload() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<40 { mjStep(m, d) }
    for i in 0..<m.ngeom {
        let viaArray = mat2Quat(d.geomXmat(i))
        let viaBuffer = d.withGeomXmat(i) { mat2Quat($0) }
        #expect(viaArray == viaBuffer)
        #expect(d.geomQuat(i) == viaArray)   // geomQuat now takes the buffer path
    }
}

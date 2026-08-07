import Testing
import CMuJoCo
@testable import MuJoCo

// Mat3 used to box nine doubles in a heap `[Double]` for every orientation read,
// and quat2Mat allocated three arrays per call (its output, its input copy, and
// Mat3's storage). Storage is now nine inline scalars. These tests pin the layout
// claim and the arithmetic that depends on the row-major element order.

@Test func mat3StorageIsInlineNotHeap() {
    // Nine contiguous Doubles and nothing else — no pointer, no refcount. If this
    // regresses to a [Double] the size drops to 8 (a single pointer).
    #expect(MemoryLayout<Mat3>.size == 9 * MemoryLayout<Double>.size)
    #expect(MemoryLayout<Mat3>.stride == 72)
    // BitwiseCopyable in practice: no ARC traffic when copied.
    #expect(_isPOD(Mat3.self))
}

@Test func mat3FlatSubscriptIsRowMajor() {
    let m = Mat3([1, 2, 3, 4, 5, 6, 7, 8, 9])
    for i in 0..<9 {
        #expect(m[i] == Double(i + 1))
    }
    #expect(m[row: 0, col: 0] == 1)
    #expect(m[row: 1, col: 2] == 6)
    #expect(m[row: 2, col: 1] == 8)
    #expect(m.array == [1, 2, 3, 4, 5, 6, 7, 8, 9])
}

@Test func mat3IdentityBehavesAsIdentity() {
    let identity = Mat3([1, 0, 0, 0, 1, 0, 0, 0, 1])
    let v = Vec3(0.3, -0.7, 1.2)
    let out = identity.times(v)
    #expect(abs(out.x - v.x) < 1e-15)
    #expect(abs(out.y - v.y) < 1e-15)
    #expect(abs(out.z - v.z) < 1e-15)
}

@Test func mat3TimesAndTransposeTimesAreConsistentWithMuJoCo() {
    // quat2Mat comes from mju_quat2Mat, so M·v must equal mju_rotVecQuat, and
    // Mᵀ·v must equal rotating by the conjugate.
    for (roll, pitch, yaw) in [(0.3, -0.2, 1.1), (-1.4, 0.9, -2.2), (0.0, 0.0, 0.0)] {
        let q = euler2Quat(roll: roll, pitch: pitch, yaw: yaw)
        let m = quat2Mat(q)
        let v = Vec3(0.4, -0.25, 1.5)

        let viaMatrix = m.times(v)
        let viaQuat = rotVecQuat(v, q)
        #expect(abs(viaMatrix.x - viaQuat.x) < 1e-12)
        #expect(abs(viaMatrix.y - viaQuat.y) < 1e-12)
        #expect(abs(viaMatrix.z - viaQuat.z) < 1e-12)

        let backViaTranspose = m.transposeTimes(v)
        let backViaConj = rotVecQuat(v, conjQuat(q))
        #expect(abs(backViaTranspose.x - backViaConj.x) < 1e-12)
        #expect(abs(backViaTranspose.y - backViaConj.y) < 1e-12)
        #expect(abs(backViaTranspose.z - backViaConj.z) < 1e-12)
    }
}

@Test func mat3ColumnsAreTheRotatedBasisVectors() {
    let q = euler2Quat(roll: 0.2, pitch: 0.4, yaw: -0.6)
    let m = quat2Mat(q)
    for (i, axis) in [Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)].enumerated() {
        let col = m.column(i)
        let rotated = rotVecQuat(axis, q)
        #expect(abs(col.x - rotated.x) < 1e-12)
        #expect(abs(col.y - rotated.y) < 1e-12)
        #expect(abs(col.z - rotated.z) < 1e-12)
    }
}

@Test func mat3EqualityIsSynthesised() {
    let a = Mat3([1, 2, 3, 4, 5, 6, 7, 8, 9])
    let b = Mat3([1, 2, 3, 4, 5, 6, 7, 8, 9])
    var c = a
    c.m11 = 99
    #expect(a == b)
    #expect(a != c)
}

@Test func mat3RoundTripsThroughArrayAndBuffer() {
    let source: [Double] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    let fromArray = Mat3(source)
    let fromBuffer = source.withUnsafeBufferPointer { Mat3($0) }
    #expect(fromArray == fromBuffer)
    #expect(fromArray.array == source)
}

// The new inline matrix accessors on MjData must agree element-for-element with
// the allocating [Double] ones they replace.

@Test func inlineMatrixAccessorsMatchTheAllocatingOnes() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    let d = MjData(m)
    for _ in 0..<40 { mjStep(m, d) }

    for i in 0..<m.ngeom {
        #expect(d.geomMatrix(i).array == d.geomXmat(i))
    }
    for i in 0..<m.nbody {
        #expect(d.bodyMatrix(i).array == d.xmat(i))
        #expect(d.bodyMatrix(i).array == d.bodyMat(i))
    }
    for i in 0..<m.nsite {
        #expect(d.siteMatrix(i).array == d.siteMat(i))
    }
    for i in 0..<m.ncam {
        #expect(d.camMatrix(i).array == d.camMat(i))
    }
}

@Test func inlineMatrixAccessorsAgreeWithTheQuaternionAccessors() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    let d = MjData(m)
    for _ in 0..<40 { mjStep(m, d) }
    for i in 0..<m.nsite {
        let viaMatrix = mat2Quat(d.siteMatrix(i).array)
        #expect(viaMatrix == d.siteQuat(i))
    }
    for i in 0..<m.ngeom {
        #expect(mat2Quat(d.geomMatrix(i).array) == d.geomQuat(i))
    }
}

// readFullState / setFullState(UnsafeBufferPointer) — the MPC-loop path that
// avoids getFullState()'s per-call allocation.

@Test func readFullStateIntoReusedBufferMatchesGetFullState() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    var scratch = [Double](repeating: 0, count: d.fullStateSize)

    for _ in 0..<5 {
        for _ in 0..<20 { mjStep(m, d) }
        d.readFullState(into: &scratch)
        #expect(scratch == d.getFullState())
    }
}

@Test func readFullStateRoundTripsThroughSetFullState() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<100 { mjStep(m, d) }

    var saved = [Double](repeating: 0, count: d.fullStateSize)
    d.readFullState(into: &saved)
    let savedTime = d.time
    let savedQpos = d.qpos

    for _ in 0..<50 { mjStep(m, d) }
    #expect(d.time != savedTime)

    d.setFullState(saved)
    #expect(d.time == savedTime)
    for (i, q) in savedQpos.enumerated() { #expect(abs(d.qpos(at: i) - q) < 1e-12) }
}

@Test func setFullStateAcceptsABorrowedBuffer() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<80 { mjStep(m, d) }
    let saved = d.getFullState()
    let savedTime = d.time

    for _ in 0..<30 { mjStep(m, d) }
    saved.withUnsafeBufferPointer { d.setFullState($0) }
    #expect(d.time == savedTime)
}

// An MPC-shaped save/rewind loop: the whole point of readFullState(into:) is that
// this reuses one buffer instead of allocating per iteration.
@Test func repeatedSaveRewindIsStable() throws {
    let m = try MjModel.load(xml: Fixtures.boxScene)
    let d = MjData(m)
    for _ in 0..<50 { mjStep(m, d) }

    var scratch = [Double](repeating: 0, count: d.fullStateSize)
    d.readFullState(into: &scratch)
    let anchorTime = d.time
    let anchorQpos = d.qpos

    for _ in 0..<25 {
        for _ in 0..<10 { mjStep(m, d) }
        d.setFullState(scratch)
        #expect(d.time == anchorTime)
    }
    for (i, q) in anchorQpos.enumerated() { #expect(abs(d.qpos(at: i) - q) < 1e-12) }
}

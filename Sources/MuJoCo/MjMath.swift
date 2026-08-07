import CMuJoCo
#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif

// `@inlinable` throughout the pure-Swift arithmetic below. Without it, `a + b` on
// a `Vec3` is a real cross-module function call from consumer code — inside a
// 200 Hz control loop, on types that exist to be cheap. Only bodies that stay
// within this module's public surface are marked; the ones that call into
// `CMuJoCo` (`mat2Quat`, `quat2Mat`, `rotVecQuat`) deliberately are not, so
// consumers never need to import the C module to inline them.

public struct Vec3: Equatable, Sendable {
    public var x, y, z: Double
    @inlinable
    public init(_ x: Double, _ y: Double, _ z: Double) { self.x = x; self.y = y; self.z = z }
    @inlinable
    public init(_ a: [Double]) {
        precondition(a.count == 3, "Vec3: expected 3 components, got \(a.count)")
        self.init(a[0], a[1], a[2])
    }
    /// Heap-allocating escape hatch. Prefer the components on a hot path.
    @inlinable
    public var array: [Double] { [x, y, z] }
    @inlinable
    public var norm: Double { (x*x + y*y + z*z).squareRoot() }
    @inlinable
    public var normalized: Vec3 { let n = norm; return n > 0 ? self * (1/n) : self }
    @inlinable
    public func dot(_ o: Vec3) -> Double { x*o.x + y*o.y + z*o.z }
    @inlinable
    public func cross(_ o: Vec3) -> Vec3 {
        Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x)
    }
    @inlinable
    public static func + (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x+b.x, a.y+b.y, a.z+b.z) }
    @inlinable
    public static func - (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x-b.x, a.y-b.y, a.z-b.z) }
    @inlinable
    public static func * (a: Vec3, s: Double) -> Vec3 { Vec3(a.x*s, a.y*s, a.z*s) }
}

/// A row-major 3x3 rotation matrix, stored **inline**.
///
/// Storage is nine scalars, not a `[Double]`: a rotation matrix is a fixed-size
/// nine doubles, and boxing that on the heap for every orientation read was pure
/// overhead on a per-geom-per-frame path.
///
/// `InlineArray` would express this more directly, but it is annotated
/// `@available(macOS 26, *)` — adopting it would force this package's deployment
/// floor from macOS 14 to macOS 26, which is a platform decision rather than a
/// storage one. Nine stored scalars give exactly the same inline layout with no
/// availability constraint, a synthesised `Equatable`, and none of `InlineArray`'s
/// missing-`Collection` friction. Swap the storage to `[9 of Double]` if the floor
/// ever moves.
///
/// Elements are reachable positionally via ``subscript(_:)`` (flat, row-major) and
/// ``subscript(row:col:)``. `array` is the `[Double]` escape hatch.
public struct Mat3: Equatable, Sendable {
    public var m00, m01, m02: Double
    public var m10, m11, m12: Double
    public var m20, m21, m22: Double

    @inlinable
    public init(m00: Double, m01: Double, m02: Double,
                m10: Double, m11: Double, m12: Double,
                m20: Double, m21: Double, m22: Double) {
        self.m00 = m00; self.m01 = m01; self.m02 = m02
        self.m10 = m10; self.m11 = m11; self.m12 = m12
        self.m20 = m20; self.m21 = m21; self.m22 = m22
    }

    /// Convenience for callers holding a `[Double]`; traps unless it has exactly
    /// 9 elements. Copies into inline storage.
    @inlinable
    public init(_ m: [Double]) {
        precondition(m.count == 9, "Mat3: expected 9 elements, got \(m.count)")
        self.init(m00: m[0], m01: m[1], m02: m[2],
                  m10: m[3], m11: m[4], m12: m[5],
                  m20: m[6], m21: m[7], m22: m[8])
    }

    /// Build from a borrowed buffer without going through `[Double]` — the path
    /// `quat2Mat` and the `mjData` matrix accessors take.
    @inlinable
    public init(_ m: UnsafeBufferPointer<Double>) {
        precondition(m.count == 9, "Mat3: expected 9 elements, got \(m.count)")
        self.init(m00: m[0], m01: m[1], m02: m[2],
                  m10: m[3], m11: m[4], m12: m[5],
                  m20: m[6], m21: m[7], m22: m[8])
    }

    /// Flat row-major element access, matching MuJoCo's `xmat` layout.
    @inlinable
    public subscript(i: Int) -> Double {
        switch i {
        case 0: return m00
        case 1: return m01
        case 2: return m02
        case 3: return m10
        case 4: return m11
        case 5: return m12
        case 6: return m20
        case 7: return m21
        case 8: return m22
        default: preconditionFailure("Mat3: index \(i) out of range 0..<9")
        }
    }

    /// Row/column element access.
    @inlinable
    public subscript(row row: Int, col col: Int) -> Double {
        precondition(row >= 0 && row < 3 && col >= 0 && col < 3,
                     "Mat3: (\(row),\(col)) out of range")
        return self[row * 3 + col]
    }

    /// Heap-allocating escape hatch for `[Double]`-shaped APIs and diagnostics.
    @inlinable
    public var array: [Double] { [m00, m01, m02, m10, m11, m12, m20, m21, m22] }

    /// Column i of the rotation matrix (e.g. column 2 = body z-axis in world).
    @inlinable
    public func column(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < 3, "Mat3.column: \(i) out of range")
        return Vec3(self[i], self[3+i], self[6+i])
    }

    /// Rᵀ · v  (world vector into body frame).
    @inlinable
    public func transposeTimes(_ v: Vec3) -> Vec3 {
        Vec3(m00*v.x + m10*v.y + m20*v.z,
             m01*v.x + m11*v.y + m21*v.z,
             m02*v.x + m12*v.y + m22*v.z)
    }

    /// Multiply this matrix (row-major) by a column vector: `M · v`.
    @inlinable
    public func times(_ v: Vec3) -> Vec3 {
        Vec3(m00*v.x + m01*v.y + m02*v.z,
             m10*v.x + m11*v.y + m12*v.z,
             m20*v.x + m21*v.y + m22*v.z)
    }
}

public struct Quat: Equatable, Sendable {
    public var w, x, y, z: Double
    @inlinable
    public init(w: Double, x: Double, y: Double, z: Double) { self.w = w; self.x = x; self.y = y; self.z = z }
}

public func mat2Quat(_ mat: [Double]) -> Quat {
    precondition(mat.count == 9)
    return mat.withUnsafeBufferPointer { mat2Quat($0) }
}

/// Row-major 3x3 rotation matrix to quaternion, reading straight from a borrowed
/// buffer. The allocation-free counterpart of ``mat2Quat(_:)-[Double]`` for
/// per-frame loops; pair it with `MjData.withGeomXmat`/`withXmat`.
public func mat2Quat(_ mat: UnsafeBufferPointer<Double>) -> Quat {
    precondition(mat.count == 9)
    var q = (0.0, 0.0, 0.0, 0.0)
    withUnsafeMutablePointer(to: &q) { qp in
        mju_mat2Quat(UnsafeMutableRawPointer(qp).assumingMemoryBound(to: Double.self),
                     mat.baseAddress)
    }
    return Quat(w: q.0, x: q.1, y: q.2, z: q.3)   // MuJoCo order wxyz
}

/// Quaternion to row-major 3x3 rotation matrix.
///
/// Allocation-free: both the output matrix and the quaternion input are fixed-size
/// tuples on the stack, handed to MuJoCo as pointers, and `Mat3`'s storage is
/// inline. This used to allocate three heap arrays per call (`m`, `qq`, and
/// `Mat3.m`).
public func quat2Mat(_ q: Quat) -> Mat3 {
    var m = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    var qq = (q.w, q.x, q.y, q.z)
    return withUnsafeMutablePointer(to: &m) { mp -> Mat3 in
        let mBase = UnsafeMutableRawPointer(mp).assumingMemoryBound(to: Double.self)
        withUnsafePointer(to: &qq) { qp in
            mju_quat2Mat(mBase, UnsafeRawPointer(qp).assumingMemoryBound(to: Double.self))
        }
        return Mat3(UnsafeBufferPointer(start: mBase, count: 9))
    }
}

extension Quat {
    /// The no-rotation quaternion.
    public static var identity: Quat { Quat(w: 1, x: 0, y: 0, z: 0) }

    /// Unit-length version of this quaternion. Returns identity for a zero quat
    /// rather than producing NaN.
    public var normalized: Quat {
        let n = (w*w + x*x + y*y + z*z).squareRoot()
        guard n > 1e-15 else { return .identity }
        return Quat(w: w/n, x: x/n, y: y/n, z: z/n)
    }
}

/// Hamilton product. `mulQuat(a, b)` is "rotate by b, then by a".
public func mulQuat(_ a: Quat, _ b: Quat) -> Quat {
    Quat(w: a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
         x: a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
         y: a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
         z: a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w)
}

/// Conjugate, which for a unit quaternion is the inverse rotation.
///
/// This is the Swift equivalent of C's `mju_negQuat`, which — despite the name —
/// conjugates rather than negating. `conjQuat` is a spelling that says so.
public func invQuat(_ q: Quat) -> Quat { Quat(w: q.w, x: -q.x, y: -q.y, z: -q.z) }

/// Conjugate. Alias for ``invQuat(_:)``, named to match what `mju_negQuat`
/// actually computes so ported C/Python code reads the same in Swift.
public func conjQuat(_ q: Quat) -> Quat { invQuat(q) }

/// Negate every component. Represents the *same* rotation as `q`; useful for
/// picking the sign with a non-negative w when comparing orientations.
///
/// - Important: This is NOT `mju_negQuat`, which conjugates — see ``conjQuat(_:)``.
public func flipQuatSign(_ q: Quat) -> Quat { Quat(w: -q.w, x: -q.x, y: -q.y, z: -q.z) }

@available(*, deprecated, message: """
    negQuat is ambiguous: it negates all four components, whereas C's mju_negQuat \
    conjugates. Use flipQuatSign(_:) for the four-component negation, or \
    conjQuat(_:)/invQuat(_:) for the mju_negQuat equivalent.
    """)
public func negQuat(_ q: Quat) -> Quat { flipQuatSign(q) }

/// Rotate a vector by a quaternion.
///
/// Delegates to `mju_rotVecQuat` rather than materialising a 3x3 matrix: the
/// matrix route allocated three heap arrays per call (`quat2Mat`'s output, its
/// input copy, and `Mat3.m`), which is not acceptable on a control loop.
public func rotVecQuat(_ v: Vec3, _ q: Quat) -> Vec3 {
    var res = (0.0, 0.0, 0.0)
    var vec = (v.x, v.y, v.z)
    var qq = (q.w, q.x, q.y, q.z)
    withUnsafeMutablePointer(to: &res) { rp in
        withUnsafeMutablePointer(to: &vec) { vp in
            withUnsafeMutablePointer(to: &qq) { qp in
                mju_rotVecQuat(
                    UnsafeMutableRawPointer(rp).assumingMemoryBound(to: Double.self),
                    UnsafeRawPointer(vp).assumingMemoryBound(to: Double.self),
                    UnsafeRawPointer(qp).assumingMemoryBound(to: Double.self))
            }
        }
    }
    return Vec3(res.0, res.1, res.2)
}

/// The rotation from `b` to `a` expressed as axis * angle (a rotation vector) in
/// **`b`'s local frame** — the same convention as C's `mju_subQuat(res, a, b)`,
/// which computes `conj(b) ⊗ a`.
///
/// - Important: This changed in the review fixes. It previously computed
///   `a ⊗ conj(b)`, a *world*-frame difference, while carrying the name of a
///   MuJoCo function with the opposite convention — so an orientation error fed
///   to a body-frame controller came out rotated. If you want the old behaviour
///   explicitly, call ``subQuatWorld(_:_:)``.
public func subQuat(_ a: Quat, _ b: Quat) -> Vec3 {
    quatToRotationVector(mulQuat(invQuat(b), a))
}

/// The rotation from `b` to `a` expressed in the **world** frame (`a ⊗ conj(b)`).
///
/// The mirror of ``subQuat(_:_:)``, which works in `b`'s local frame. Both
/// return the same magnitude; they differ in the frame the axis is expressed in.
public func subQuatWorld(_ a: Quat, _ b: Quat) -> Vec3 {
    quatToRotationVector(mulQuat(a, invQuat(b)))
}

/// Convert a rotation quaternion to a rotation vector (axis * angle), taking the
/// shortest arc. Equivalent to `mju_quat2Vel(res, q, 1)`.
public func quatToRotationVector(_ q: Quat) -> Vec3 {
    var d = q.normalized
    if d.w < 0 { d = flipQuatSign(d) }     // shortest arc
    let sinHalf = (d.x*d.x + d.y*d.y + d.z*d.z).squareRoot()
    guard sinHalf > 1e-15 else { return Vec3(0, 0, 0) }
    let angle = 2 * atan2(sinHalf, d.w)
    let s = angle / sinHalf
    return Vec3(d.x * s, d.y * s, d.z * s)
}

/// Intrinsic Z-Y-X (yaw-pitch-roll) Euler angles from a quaternion, radians.
public func quat2Euler(_ q: Quat) -> (roll: Double, pitch: Double, yaw: Double) {
    let n = q.normalized
    let sinp = 2 * (n.w*n.y - n.z*n.x)
    let pitch = abs(sinp) >= 1 ? (sinp > 0 ? Double.pi/2 : -Double.pi/2) : asin(sinp)
    let roll = atan2(2 * (n.w*n.x + n.y*n.z), 1 - 2 * (n.x*n.x + n.y*n.y))
    let yaw  = atan2(2 * (n.w*n.z + n.x*n.y), 1 - 2 * (n.y*n.y + n.z*n.z))
    return (roll, pitch, yaw)
}

/// Quaternion from intrinsic Z-Y-X (yaw-pitch-roll) Euler angles, radians.
public func euler2Quat(roll: Double, pitch: Double, yaw: Double) -> Quat {
    let (cr, sr) = (cos(roll/2), sin(roll/2))
    let (cp, sp) = (cos(pitch/2), sin(pitch/2))
    let (cy, sy) = (cos(yaw/2), sin(yaw/2))
    return Quat(w: cr*cp*cy + sr*sp*sy,
                x: sr*cp*cy - cr*sp*sy,
                y: cr*sp*cy + sr*cp*sy,
                z: cr*cp*sy - sr*sp*cy)
}

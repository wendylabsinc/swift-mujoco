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

/// A row-major 3x3 rotation matrix, stored **inline** in an `InlineArray`.
///
/// A rotation matrix is a fixed-size nine doubles. Storing it in a `[Double]` put
/// it on the heap and cost an allocation for every orientation read, on a
/// per-geom-per-frame path; `InlineArray` gives fixed-size storage with no
/// indirection and no refcount.
///
/// Two consequences of `InlineArray` worth knowing, both handled here:
/// it is **not a `Collection`** (no `map`/`reduce`/`for…in` over ``m`` — index it,
/// or use ``span``/``array``), and it does **not conform to `Equatable`**, so `==`
/// is written out below rather than synthesised.
public struct Mat3: Equatable, Sendable {
    /// Row-major, 9 elements. Indexable but not a `Collection`.
    public let m: [9 of Double]

    @inlinable
    public init(_ m: [9 of Double]) { self.m = m }

    @inlinable
    public init(m00: Double, m01: Double, m02: Double,
                m10: Double, m11: Double, m12: Double,
                m20: Double, m21: Double, m22: Double) {
        self.m = [m00, m01, m02, m10, m11, m12, m20, m21, m22]
    }

    /// Convenience for callers holding a `[Double]`; traps unless it has exactly
    /// 9 elements. Copies into inline storage.
    @inlinable
    public init(_ m: [Double]) {
        precondition(m.count == 9, "Mat3: expected 9 elements, got \(m.count)")
        self.init([9 of Double] { m[$0] })
    }

    /// Build from a borrowed buffer without going through `[Double]` — the path
    /// `quat2Mat` and the `mjData` matrix accessors take.
    @inlinable
    public init(_ m: UnsafeBufferPointer<Double>) {
        precondition(m.count == 9, "Mat3: expected 9 elements, got \(m.count)")
        self.init([9 of Double] { m[$0] })
    }

    /// Flat row-major element access, matching MuJoCo's `xmat` layout.
    @inlinable
    public subscript(i: Int) -> Double { m[i] }

    /// Row/column element access.
    @inlinable
    public subscript(row row: Int, col col: Int) -> Double {
        precondition(row >= 0 && row < 3 && col >= 0 && col < 3,
                     "Mat3: (\(row),\(col)) out of range")
        return m[row * 3 + col]
    }

    /// Borrowed view of the nine elements, for code that wants to iterate without
    /// copying. `InlineArray` is not a `Collection`, but its `Span` gives you
    /// `indices` and a subscript.
    @inlinable
    public var span: Span<Double> {
        @_lifetime(borrow self) get { m.span }
    }

    /// Heap-allocating escape hatch for `[Double]`-shaped APIs and diagnostics.
    @inlinable
    public var array: [Double] { [m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]] }

    /// Column i of the rotation matrix (e.g. column 2 = body z-axis in world).
    @inlinable
    public func column(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < 3, "Mat3.column: \(i) out of range")
        return Vec3(m[i], m[3+i], m[6+i])
    }

    /// Rᵀ · v  (world vector into body frame).
    @inlinable
    public func transposeTimes(_ v: Vec3) -> Vec3 {
        Vec3(m[0]*v.x + m[3]*v.y + m[6]*v.z,
             m[1]*v.x + m[4]*v.y + m[7]*v.z,
             m[2]*v.x + m[5]*v.y + m[8]*v.z)
    }

    /// Multiply this matrix (row-major) by a column vector: `M · v`.
    @inlinable
    public func times(_ v: Vec3) -> Vec3 {
        Vec3(m[0]*v.x + m[1]*v.y + m[2]*v.z,
             m[3]*v.x + m[4]*v.y + m[5]*v.z,
             m[6]*v.x + m[7]*v.y + m[8]*v.z)
    }

    /// Written out because `InlineArray` has no `Equatable` conformance.
    @inlinable
    public static func == (a: Mat3, b: Mat3) -> Bool {
        for i in 0..<9 where a.m[i] != b.m[i] { return false }
        return true
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

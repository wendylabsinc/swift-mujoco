import CMuJoCo
#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif

public struct Vec3: Equatable, Sendable {
    public var x, y, z: Double
    public init(_ x: Double, _ y: Double, _ z: Double) { self.x = x; self.y = y; self.z = z }
    public init(_ a: [Double]) { self.init(a[0], a[1], a[2]) }
    public var array: [Double] { [x, y, z] }
    public var norm: Double { (x*x + y*y + z*z).squareRoot() }
    public var normalized: Vec3 { let n = norm; return n > 0 ? self * (1/n) : self }
    public func dot(_ o: Vec3) -> Double { x*o.x + y*o.y + z*o.z }
    public func cross(_ o: Vec3) -> Vec3 {
        Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x)
    }
    public static func + (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x+b.x, a.y+b.y, a.z+b.z) }
    public static func - (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x-b.x, a.y-b.y, a.z-b.z) }
    public static func * (a: Vec3, s: Double) -> Vec3 { Vec3(a.x*s, a.y*s, a.z*s) }
}

public struct Mat3: Equatable, Sendable {
    public let m: [Double]   // 9, row-major
    public init(_ m: [Double]) { precondition(m.count == 9); self.m = m }
    /// Column i of the rotation matrix (e.g. column 2 = body z-axis in world).
    public func column(_ i: Int) -> Vec3 { Vec3(m[i], m[3+i], m[6+i]) }
    /// Rᵀ · v  (world vector into body frame).
    public func transposeTimes(_ v: Vec3) -> Vec3 {
        Vec3(m[0]*v.x + m[3]*v.y + m[6]*v.z,
             m[1]*v.x + m[4]*v.y + m[7]*v.z,
             m[2]*v.x + m[5]*v.y + m[8]*v.z)
    }
    /// Multiply this matrix (row-major) by a column vector: `M · v`.
    public func times(_ v: Vec3) -> Vec3 {
        Vec3(m[0]*v.x + m[1]*v.y + m[2]*v.z,
             m[3]*v.x + m[4]*v.y + m[5]*v.z,
             m[6]*v.x + m[7]*v.y + m[8]*v.z)
    }
}

public struct Quat: Equatable, Sendable {
    public var w, x, y, z: Double
    public init(w: Double, x: Double, y: Double, z: Double) { self.w = w; self.x = x; self.y = y; self.z = z }
}

public func mat2Quat(_ mat: [Double]) -> Quat {
    precondition(mat.count == 9)
    var q = [Double](repeating: 0, count: 4)
    mat.withUnsafeBufferPointer { mp in
        mju_mat2Quat(&q, mp.baseAddress)
    }
    return Quat(w: q[0], x: q[1], y: q[2], z: q[3])   // MuJoCo order wxyz
}

public func quat2Mat(_ q: Quat) -> Mat3 {
    var m = [Double](repeating: 0, count: 9)
    var qq = [q.w, q.x, q.y, q.z]
    mju_quat2Mat(&m, &qq)
    return Mat3(m)
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
public func invQuat(_ q: Quat) -> Quat { Quat(w: q.w, x: -q.x, y: -q.y, z: -q.z) }

/// Negate every component. Represents the *same* rotation as `q`; useful for
/// picking the sign with a non-negative w when comparing orientations.
public func negQuat(_ q: Quat) -> Quat { Quat(w: -q.w, x: -q.x, y: -q.y, z: -q.z) }

/// Rotate a vector by a quaternion.
public func rotVecQuat(_ v: Vec3, _ q: Quat) -> Vec3 {
    quat2Mat(q).times(v)
}

/// The rotation taking `b` to `a`, expressed as axis * angle (a rotation vector).
public func subQuat(_ a: Quat, _ b: Quat) -> Vec3 {
    var d = mulQuat(a, invQuat(b)).normalized
    if d.w < 0 { d = negQuat(d) }          // shortest arc
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

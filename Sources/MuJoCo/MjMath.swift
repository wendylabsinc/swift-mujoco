import CMuJoCo

public struct Vec3: Equatable {
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

public struct Mat3 {
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
}

public struct Quat: Equatable {
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

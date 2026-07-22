import CMuJoCo

public final class MjData {
    public let ptr: UnsafeMutablePointer<mjData>
    let model: MjModel   // keep the model alive for this data's lifetime

    public init(_ model: MjModel) {
        self.model = model
        self.ptr = mj_makeData(model.ptr)
    }
    deinit { mj_deleteData(ptr) }

    public var time: Double { ptr.pointee.time }

    private func buffer(_ base: UnsafeMutablePointer<mjtNum>?, _ n: Int) -> [Double] {
        guard let base, n > 0 else { return [] }
        return Array(UnsafeBufferPointer(start: base, count: n))
    }

    public var qpos: [Double] { buffer(ptr.pointee.qpos, model.nq) }
    public var qvel: [Double] { buffer(ptr.pointee.qvel, model.nv) }
    public var ctrl: [Double] { buffer(ptr.pointee.ctrl, model.nu) }

    public func setCtrl(_ index: Int, _ value: Double) {
        precondition(index >= 0 && index < model.nu)
        ptr.pointee.ctrl[index] = value
    }
    public func setCtrl(_ values: [Double]) {
        for i in 0..<min(values.count, model.nu) { ptr.pointee.ctrl[i] = values[i] }
    }

    public func geomXpos(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < model.ngeom)
        let b = ptr.pointee.geom_xpos!   // mjtNum*, length ngeom*3
        return Vec3(b[i*3+0], b[i*3+1], b[i*3+2])
    }
    public func geomXmat(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < model.ngeom)
        let b = ptr.pointee.geom_xmat!   // mjtNum*, length ngeom*9, row-major
        return (0..<9).map { b[i*9 + $0] }
    }
    public func geomQuat(_ i: Int) -> Quat { mat2Quat(geomXmat(i)) }

    public struct Contact {
        public let geom1: Int, geom2: Int
        public let dist: Double
        public let pos: Vec3
        public let forceNormal: Double
    }

    public var ncon: Int { Int(ptr.pointee.ncon) }

    public func contacts(max: Int = 64) -> [Contact] {
        let n = Swift.min(ncon, max)
        guard n > 0 else { return [] }
        var out: [Contact] = []
        out.reserveCapacity(n)
        var f6 = [Double](repeating: 0, count: 6)
        for i in 0..<n {
            let con = ptr.pointee.contact[i]
            mj_contactForce(model.ptr, ptr, Int32(i), &f6)
            let p = con.pos   // mjtNum[3] tuple
            out.append(Contact(geom1: Int(con.geom1), geom2: Int(con.geom2),
                               dist: con.dist, pos: Vec3(p.0, p.1, p.2),
                               forceNormal: f6[0]))
        }
        return out
    }
}

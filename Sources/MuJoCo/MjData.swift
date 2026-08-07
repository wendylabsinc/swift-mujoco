import CMuJoCo

/// Intentionally NOT `Sendable`: this class wraps mutable MuJoCo C state
/// (`mjData`) and must stay on a single isolation domain — stepping and
/// other MuJoCo APIs mutate shared state that is not safe to touch
/// concurrently from multiple isolation domains.
public final class MjData {
    /// Deliberate low-level escape hatch for downstream C interop with the
    /// raw `mjData`. The library retains ownership of this pointer (it is
    /// freed in `deinit`); callers must NOT free it or hand it to another
    /// wrapper that assumes ownership.
    public let ptr: UnsafeMutablePointer<mjData>
    let model: MjModel   // keep the model alive for this data's lifetime

    public init(_ model: MjModel) {
        self.model = model
        guard let p = mj_makeData(model.ptr) else {
            // mj_makeData only returns NULL when allocation fails (it routes
            // other problems through mju_error). Trap here with a message rather
            // than storing NULL and crashing later inside an accessor.
            preconditionFailure("mj_makeData returned NULL: out of memory for a model with nq=\(model.nq), nv=\(model.nv)")
        }
        self.ptr = p
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
        precondition(values.count == model.nu, "setCtrl: expected \(model.nu) values, got \(values.count)")
        for i in 0..<model.nu { ptr.pointee.ctrl[i] = values[i] }
    }

    /// Non-allocating view of `qpos`, valid only for the duration of `body`.
    /// This is the 200 Hz path; `qpos` allocates a fresh Array on every access.
    public func withQpos<R>(_ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        guard let base = ptr.pointee.qpos else { return body(UnsafeBufferPointer(start: nil, count: 0)) }
        return body(UnsafeBufferPointer(start: base, count: model.nq))
    }

    /// Non-allocating view of `qvel`, valid only for the duration of `body`.
    public func withQvel<R>(_ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        guard let base = ptr.pointee.qvel else { return body(UnsafeBufferPointer(start: nil, count: 0)) }
        return body(UnsafeBufferPointer(start: base, count: model.nv))
    }

    /// Non-allocating view of `ctrl`, valid only for the duration of `body`.
    public func withCtrl<R>(_ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        guard let base = ptr.pointee.ctrl else { return body(UnsafeBufferPointer(start: nil, count: 0)) }
        return body(UnsafeBufferPointer(start: base, count: model.nu))
    }

    /// One generalized position, without copying the whole vector.
    public func qpos(at i: Int) -> Double {
        precondition(i >= 0 && i < model.nq)
        return ptr.pointee.qpos[i]
    }

    /// One generalized velocity, without copying the whole vector.
    public func qvel(at i: Int) -> Double {
        precondition(i >= 0 && i < model.nv)
        return ptr.pointee.qvel[i]
    }

    /// Write one generalized position directly. Call `mjForward` afterwards for
    /// dependent quantities to catch up.
    public func setQpos(at i: Int, _ value: Double) {
        precondition(i >= 0 && i < model.nq)
        ptr.pointee.qpos[i] = value
    }

    /// Write one generalized velocity directly.
    public func setQvel(at i: Int, _ value: Double) {
        precondition(i >= 0 && i < model.nv)
        ptr.pointee.qvel[i] = value
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

    /// Non-allocating view of one geom's row-major 3x3 world orientation, valid
    /// only for the duration of `body`. `geomXmat(_:)` allocates a fresh
    /// 9-element Array per call, which a per-geom-per-frame loop cannot afford.
    public func withGeomXmat<R>(_ i: Int, _ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        precondition(i >= 0 && i < model.ngeom)
        let b = ptr.pointee.geom_xmat!
        return body(UnsafeBufferPointer(start: b + i*9, count: 9))
    }

    /// World orientation of a geom as an inline-stored ``Mat3``. Allocation-free
    /// counterpart of `geomXmat(_:)`.
    public func geomMatrix(_ i: Int) -> Mat3 {
        precondition(i >= 0 && i < model.ngeom)
        let b = ptr.pointee.geom_xmat!
        return Mat3(UnsafeBufferPointer(start: b + i*9, count: 9))
    }

    /// World orientation of a geom as a quaternion.
    ///
    /// Reads straight out of `mjData` and converts in place — no intermediate
    /// `[Double]`/`Mat3` allocation, unlike the `mat2Quat(geomXmat(i))` this
    /// replaces (three heap arrays per geom, per frame).
    public func geomQuat(_ i: Int) -> Quat {
        withGeomXmat(i) { mat2Quat($0) }
    }

    public struct Contact: Sendable {
        public let geom1: Int, geom2: Int
        public let dist: Double
        public let pos: Vec3
        public let forceNormal: Double
    }

    public var ncon: Int { Int(ptr.pointee.ncon) }

    /// Whether `contacts(max:)` with this cap would drop contacts. Compare
    /// `ncon` against the cap yourself for the count; this is the cheap
    /// "did I lose data?" check for a caller that only needs the boolean.
    public func contactsTruncated(max: Int = 64) -> Bool { ncon > max }

    /// Up to `max` contacts. A contact-rich scene has more than the default cap;
    /// the excess is dropped silently, so check ``contactsTruncated(max:)`` (or
    /// compare ``ncon``) when completeness matters.
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

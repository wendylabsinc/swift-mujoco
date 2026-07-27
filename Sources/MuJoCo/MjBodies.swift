import CMuJoCo

extension MjModel {
    public var nsite: Int { Int(ptr.pointee.nsite) }
    public var ncam: Int { Int(ptr.pointee.ncam) }
}

extension MjData {
    /// Cartesian position of a body's frame origin, world coordinates.
    ///
    /// - Note: Despite the generic-looking name, this is body-specific and
    ///   bounds-checks against `model.nbody` — passing a site or geom id here
    ///   silently reads an unrelated body's pose whenever that id happens to
    ///   be `< nbody`. Prefer the unambiguous `bodyPos(_:)`, which forwards
    ///   to this same implementation.
    public func xpos(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < model.nbody)
        let b = ptr.pointee.xpos!            // mjtNum*, nbody*3
        return Vec3(b[i*3+0], b[i*3+1], b[i*3+2])
    }

    /// Orientation of a body's frame as a quaternion, MuJoCo (w,x,y,z) order.
    ///
    /// - Note: Body-specific despite the generic name; see `xpos(_:)`.
    ///   Prefer `bodyQuat(_:)`, which forwards to this same implementation.
    public func xquat(_ i: Int) -> Quat {
        precondition(i >= 0 && i < model.nbody)
        let b = ptr.pointee.xquat!           // mjtNum*, nbody*4
        return Quat(w: b[i*4+0], x: b[i*4+1], y: b[i*4+2], z: b[i*4+3])
    }

    /// Orientation of a body's frame as a row-major 3x3 matrix.
    ///
    /// - Note: Body-specific despite the generic name; see `xpos(_:)`.
    ///   Prefer `bodyMat(_:)`, which forwards to this same implementation.
    public func xmat(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < model.nbody)
        let b = ptr.pointee.xmat!            // mjtNum*, nbody*9
        return (0..<9).map { b[i*9 + $0] }
    }

    /// Cartesian position of a body's frame origin, world coordinates.
    /// Clearly-named forward of `xpos(_:)`, which sites and geoms would
    /// otherwise ambiguously share the name of (`sitePos`/`geomXpos` already
    /// disambiguate; `xpos` alone historically did not).
    public func bodyPos(_ i: Int) -> Vec3 { xpos(i) }

    /// Orientation of a body's frame as a quaternion, MuJoCo (w,x,y,z) order.
    /// Clearly-named forward of `xquat(_:)`.
    public func bodyQuat(_ i: Int) -> Quat { xquat(i) }

    /// Orientation of a body's frame as a row-major 3x3 matrix.
    /// Clearly-named forward of `xmat(_:)`.
    public func bodyMat(_ i: Int) -> [Double] { xmat(i) }

    /// World position of a site — where IMUs and rangefinders are mounted.
    public func sitePos(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < model.nsite)
        let b = ptr.pointee.site_xpos!       // mjtNum*, nsite*3
        return Vec3(b[i*3+0], b[i*3+1], b[i*3+2])
    }

    /// World orientation of a site as a row-major 3x3 matrix.
    public func siteMat(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < model.nsite)
        let b = ptr.pointee.site_xmat!       // mjtNum*, nsite*9
        return (0..<9).map { b[i*9 + $0] }
    }

    /// World orientation of a site as a quaternion.
    public func siteQuat(_ i: Int) -> Quat { mat2Quat(siteMat(i)) }

    /// World position of a camera.
    public func camPos(_ i: Int) -> Vec3 {
        precondition(i >= 0 && i < model.ncam)
        let b = ptr.pointee.cam_xpos!        // mjtNum*, ncam*3
        return Vec3(b[i*3+0], b[i*3+1], b[i*3+2])
    }

    /// World orientation of a camera as a row-major 3x3 matrix.
    public func camMat(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < model.ncam)
        let b = ptr.pointee.cam_xmat!        // mjtNum*, ncam*9
        return (0..<9).map { b[i*9 + $0] }
    }

    /// World orientation of a camera as a quaternion. Sites and geoms both
    /// have a quaternion accessor (`siteQuat`/`geomQuat`); this fills the
    /// same gap for cameras.
    public func camQuat(_ i: Int) -> Quat { mat2Quat(camMat(i)) }

    /// Generalized accelerations. Allocates; use `withQacc` on the hot path.
    public var qacc: [Double] {
        guard let base = ptr.pointee.qacc, model.nv > 0 else { return [] }
        return Array(UnsafeBufferPointer(start: base, count: model.nv))
    }

    /// Non-allocating view of `qacc`, valid only for the duration of `body`.
    public func withQacc<R>(_ body: (UnsafeBufferPointer<Double>) -> R) -> R {
        guard let base = ptr.pointee.qacc else {
            return body(UnsafeBufferPointer(start: nil, count: 0))
        }
        return body(UnsafeBufferPointer(start: base, count: model.nv))
    }

    /// Apply an external Cartesian force and torque to a body, world frame.
    /// Persists until overwritten or `mj_resetData` — set it to zero to clear.
    public func setXfrcApplied(body i: Int, force: Vec3, torque: Vec3) {
        precondition(i >= 0 && i < model.nbody)
        let b = ptr.pointee.xfrc_applied!   // mjtNum*, nbody*6 (force then torque)
        b[i*6+0] = force.x;  b[i*6+1] = force.y;  b[i*6+2] = force.z
        b[i*6+3] = torque.x; b[i*6+4] = torque.y; b[i*6+5] = torque.z
    }
}

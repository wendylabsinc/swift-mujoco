import CMuJoCo

/// Procedural scene composition: build a model in code, then compile it to an `MjModel`.
///
/// Intentionally NOT `Sendable`: this class wraps mutable MuJoCo C state
/// (`mjSpec`) and must stay on a single isolation domain — mutating APIs
/// touch shared state that is not safe to access concurrently from
/// multiple isolation domains.
public final class MjSpec {
    /// Deliberate low-level escape hatch for downstream C interop with the
    /// raw `mjSpec`. The library retains ownership of this pointer (it is
    /// freed in `deinit`); callers must NOT free it or hand it to another
    /// wrapper that assumes ownership.
    public let ptr: UnsafeMutablePointer<mjSpec>

    public init(floor: Bool = true, light: Bool = true) {
        self.ptr = mj_makeSpec()
        let world = mjs_findBody(ptr, "world")
        if light {
            _ = mjs_addLight(world, nil)
        }
        if floor {
            let g = mjs_addGeom(world, nil)
            g!.pointee.type = mjGEOM_PLANE
            g!.pointee.size = (12, 12, 0.1)
            g!.pointee.rgba = (0.26, 0.27, 0.32, 1)
        }
    }

    deinit { mj_deleteSpec(ptr) }

    @discardableResult
    public func addBody(name: String, pos: [Double]) -> String {
        let world = mjs_findBody(ptr, "world")
        let b = mjs_addBody(world, nil)
        _ = mjs_setName(b!.pointee.element, name)
        b!.pointee.pos = (pos[0], pos[1], pos[2])
        return name
    }

    public func addGeom(type: MjModel.GeomType, size: [Double], pos: [Double],
                         rgba: [Double], toBody body: String? = nil) {
        let parent = mjs_findBody(ptr, body ?? "world")
        precondition(parent != nil, "addGeom: no body named \"\(body ?? "world")\" in this spec")
        let g = mjs_addGeom(parent, nil)
        g!.pointee.type = cGeomType(type)
        g!.pointee.size = (size[0], size[1], size[2])
        g!.pointee.pos = (pos[0], pos[1], pos[2])
        g!.pointee.rgba = (Float(rgba[0]), Float(rgba[1]), Float(rgba[2]), Float(rgba[3]))
    }

    public func compile() throws -> MjModel {
        guard let m = mj_compile(ptr, nil) else {
            throw MjError("mj_compile failed: " + String(cString: mjs_getError(ptr)))
        }
        return MjModel(owning: m)
    }

    private func cGeomType(_ t: MjModel.GeomType) -> mjtGeom {
        switch t {
        case .plane: return mjGEOM_PLANE
        case .sphere: return mjGEOM_SPHERE
        case .capsule: return mjGEOM_CAPSULE
        case .ellipsoid: return mjGEOM_ELLIPSOID
        case .cylinder: return mjGEOM_CYLINDER
        case .box, .mesh, .other: return mjGEOM_BOX
        }
    }
}

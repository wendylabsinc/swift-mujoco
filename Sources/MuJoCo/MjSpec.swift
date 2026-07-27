import CMuJoCo
import Foundation

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

    /// Private initializer for a spec we did not create with `mj_makeSpec`
    /// (e.g. one produced by `mj_parseXML`). We still own it and must free it.
    private init(owning ptr: UnsafeMutablePointer<mjSpec>) { self.ptr = ptr }

    /// Parse an existing MJCF file into an editable spec.
    public convenience init(xmlPath: String) throws {
        var err = [CChar](repeating: 0, count: 1000)
        guard let s = mj_parseXML(xmlPath, nil, &err, Int32(err.count)) else {
            throw MjError(err.withUnsafeBufferPointer { String(cString: $0.baseAddress!) })
        }
        self.init(owning: s)
    }

    /// Parse an MJCF string into an editable spec.
    public convenience init(xml: String) throws {
        let dir = FileManager.default.temporaryDirectory
        let file = dir.appendingPathComponent("mjspec-\(UUID().uuidString).xml")
        try xml.write(to: file, atomically: true, encoding: .utf8)
        defer { try? FileManager.default.removeItem(at: file) }
        try self.init(xmlPath: file.path)
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

    /// Graft another spec's body tree into this one under a name prefix.
    ///
    /// This is what makes multi-robot composition possible without XML
    /// `<replicate>`/`<include>` tricks: every body, joint, geom, site, camera
    /// and sensor from `child`'s worldbody gains `prefix` (and `suffix`), so
    /// two copies of the same robot no longer collide on names.
    ///
    /// `child` must outlive this call but is not consumed — MuJoCo copies
    /// from it into this spec.
    public func attach(_ child: MjSpec, prefix: String, suffix: String = "",
                        toBody body: String = "world") throws {
        guard let parent = mjs_findBody(ptr, body) else {
            throw MjError("attach: no body named \"\(body)\" in the parent spec")
        }
        guard let frame = mjs_addFrame(parent, nil) else {
            throw MjError("attach: could not add an attachment frame to \"\(body)\"")
        }
        guard let childWorld = mjs_findBody(child.ptr, "world") else {
            throw MjError("attach: child spec has no worldbody")
        }
        guard mjs_attach(frame.pointee.element, childWorld.pointee.element,
                          prefix, suffix) != nil else {
            throw MjError("attach failed: " + String(cString: mjs_getError(ptr)))
        }
    }

    /// Add a site — where IMUs, rangefinders and touch sensors mount.
    public func addSite(name: String, pos: [Double], size: Double = 0.01,
                         toBody body: String = "world") {
        let parent = mjs_findBody(ptr, body)
        precondition(parent != nil, "addSite: no body named \"\(body)\" in this spec")
        let s = mjs_addSite(parent, nil)
        _ = mjs_setName(s!.pointee.element, name)
        s!.pointee.pos = (pos[0], pos[1], pos[2])
        s!.pointee.size = (size, size, size)
    }

    /// Add a fixed camera.
    public func addCamera(name: String, pos: [Double], fovy: Double,
                           toBody body: String = "world") {
        let parent = mjs_findBody(ptr, body)
        precondition(parent != nil, "addCamera: no body named \"\(body)\" in this spec")
        let c = mjs_addCamera(parent, nil)
        _ = mjs_setName(c!.pointee.element, name)
        c!.pointee.pos = (pos[0], pos[1], pos[2])
        c!.pointee.fovy = fovy
    }

    /// Every body name in this spec, world first.
    ///
    /// Implemented via compile-and-inspect rather than walking `mjsBody`
    /// children directly: MuJoCo's spec tree-walking API (`mjs_nextChild`)
    /// takes an `mjsBody*` as its first argument, not an `mjsElement*`, which
    /// does not compose cleanly here without extra unwrapping across 3.x
    /// minors. Compiling a throwaway copy is correct and adequate for what
    /// callers need this for (name lookups before/after attach).
    public func findBodyNames() -> [String] {
        guard let m = try? compileCopy() else { return [] }
        return m.bodyNames
    }

    /// Compile without consuming this spec, for read-only inspection.
    private func compileCopy() throws -> MjModel {
        guard let copy = mj_copySpec(ptr) else { throw MjError("mj_copySpec failed") }
        defer { mj_deleteSpec(copy) }
        guard let m = mj_compile(copy, nil) else {
            throw MjError("mj_compile failed: " + String(cString: mjs_getError(copy)))
        }
        return MjModel(owning: m)
    }

    /// Serialize this spec back to MJCF.
    ///
    /// `mj_saveXMLString` refuses an uncompiled spec ("Only compiled model
    /// can be written"), so this compiles a throwaway copy first — `self`
    /// is left untouched and remains editable afterwards.
    public func saveXML() throws -> String {
        guard let copy = mj_copySpec(ptr) else { throw MjError("mj_copySpec failed") }
        defer { mj_deleteSpec(copy) }
        guard let m = mj_compile(copy, nil) else {
            throw MjError("mj_compile failed: " + String(cString: mjs_getError(copy)))
        }
        defer { mj_deleteModel(m) }
        var buf = [CChar](repeating: 0, count: 1 << 20)
        var err = [CChar](repeating: 0, count: 1000)
        guard mj_saveXMLString(copy, &buf, Int32(buf.count), &err, Int32(err.count)) >= 0 else {
            throw MjError(err.withUnsafeBufferPointer { String(cString: $0.baseAddress!) })
        }
        return buf.withUnsafeBufferPointer { String(cString: $0.baseAddress!) }
    }

    private func cGeomType(_ t: MjModel.GeomType) -> mjtGeom {
        switch t {
        case .plane: return mjGEOM_PLANE
        case .sphere: return mjGEOM_SPHERE
        case .capsule: return mjGEOM_CAPSULE
        case .ellipsoid: return mjGEOM_ELLIPSOID
        case .cylinder: return mjGEOM_CYLINDER
        case .box: return mjGEOM_BOX
        case .mesh: return mjGEOM_MESH
        case .other: return mjGEOM_BOX
        }
    }
}

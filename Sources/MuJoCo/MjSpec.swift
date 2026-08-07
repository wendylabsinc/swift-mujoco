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
    ///
    /// - Warning: `attach(_:prefix:suffix:toBody:)` retains its child in
    ///   `attachedChildren` specifically so the child's `mjSpec*` outlives
    ///   every later `compile()`/`saveXML()`/`findBodyNames()` on the parent
    ///   (`mjs_attach` links by reference; the merge happens lazily inside
    ///   `mj_compile`, reading live from the child). Calling `mjs_attach`
    ///   directly through this `ptr` — bypassing `MjSpec.attach` — skips that
    ///   retain and reintroduces the same use-after-free it exists to
    ///   prevent. Separately, a deliberate mutual attach between two
    ///   `MjSpec`s (A retains B via `attachedChildren` and B retains A) would
    ///   create a retain cycle and leak both; nothing here detects that.
    public let ptr: UnsafeMutablePointer<mjSpec>

    /// Children linked into this spec via `attach`, retained for as long as
    /// this spec is. See `attach` for why this is required, not optional.
    private var attachedChildren: [MjSpec] = []

    public init(floor: Bool = true, light: Bool = true) {
        guard let p = mj_makeSpec() else {
            preconditionFailure("mj_makeSpec returned NULL (out of memory)")
        }
        self.ptr = p
        guard let world = mjs_findBody(ptr, "world") else {
            preconditionFailure("a fresh mjSpec has no \"world\" body; MuJoCo version mismatch?")
        }
        if light {
            _ = mjs_addLight(world, nil)
        }
        if floor {
            guard let g = mjs_addGeom(world, nil) else {
                preconditionFailure("mjs_addGeom returned NULL adding the default floor")
            }
            g.pointee.type = mjGEOM_PLANE
            g.pointee.size = (12, 12, 0.1)
            g.pointee.rgba = (0.26, 0.27, 0.32, 1)
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
        precondition(pos.count == 3, "addBody: pos needs 3 components, got \(pos.count)")
        guard let world = mjs_findBody(ptr, "world") else {
            preconditionFailure("addBody: this spec has no \"world\" body")
        }
        guard let b = mjs_addBody(world, nil) else {
            preconditionFailure("addBody: mjs_addBody returned NULL")
        }
        _ = mjs_setName(b.pointee.element, name)
        b.pointee.pos = (pos[0], pos[1], pos[2])
        return name
    }

    public func addGeom(type: MjModel.GeomType, size: [Double], pos: [Double],
                         rgba: [Double], toBody body: String? = nil) {
        // `.other` is the read-side catch-all for geom types this wrapper does not
        // model; it has no meaningful write-side mapping. It used to silently
        // become a box, which quietly produced a different model than asked for.
        precondition(type != .other,
                     "addGeom: .other is not a constructible geom type (it is the read-side catch-all)")
        precondition(size.count == 3, "addGeom: size needs 3 components, got \(size.count)")
        precondition(pos.count == 3, "addGeom: pos needs 3 components, got \(pos.count)")
        precondition(rgba.count == 4, "addGeom: rgba needs 4 components, got \(rgba.count)")
        let parent = mjs_findBody(ptr, body ?? "world")
        precondition(parent != nil, "addGeom: no body named \"\(body ?? "world")\" in this spec")
        guard let g = mjs_addGeom(parent, nil) else {
            preconditionFailure("addGeom: mjs_addGeom returned NULL")
        }
        g.pointee.type = cGeomType(type)
        g.pointee.size = (size[0], size[1], size[2])
        g.pointee.pos = (pos[0], pos[1], pos[2])
        g.pointee.rgba = (Float(rgba[0]), Float(rgba[1]), Float(rgba[2]), Float(rgba[3]))
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
    /// `mjs_attach` links the child in **by reference, not by copy**: the
    /// actual merge is deferred until `mj_compile` runs, and that compile
    /// reads live from `child`'s `mjSpec`. `child`'s underlying `mjSpec*`
    /// therefore must stay alive until this spec is done with it (including
    /// through every later `compile()`/`saveXML()`/`findBodyNames()` call).
    /// This method retains `child` for exactly that reason, so callers may
    /// safely pass a temporary that they otherwise hold no reference to.
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
        // Tie child's lifetime to ours: mjs_attach links by reference, and
        // mj_compile reads live from child.ptr, so it must outlive us.
        attachedChildren.append(child)
    }

    /// Add a site — where IMUs, rangefinders and touch sensors mount.
    public func addSite(name: String, pos: [Double], size: Double = 0.01,
                         toBody body: String = "world") {
        precondition(pos.count == 3, "addSite: pos needs 3 components, got \(pos.count)")
        let parent = mjs_findBody(ptr, body)
        precondition(parent != nil, "addSite: no body named \"\(body)\" in this spec")
        guard let s = mjs_addSite(parent, nil) else {
            preconditionFailure("addSite: mjs_addSite returned NULL")
        }
        _ = mjs_setName(s.pointee.element, name)
        s.pointee.pos = (pos[0], pos[1], pos[2])
        s.pointee.size = (size, size, size)
    }

    /// Add a fixed camera.
    public func addCamera(name: String, pos: [Double], fovy: Double,
                           toBody body: String = "world") {
        precondition(pos.count == 3, "addCamera: pos needs 3 components, got \(pos.count)")
        let parent = mjs_findBody(ptr, body)
        precondition(parent != nil, "addCamera: no body named \"\(body)\" in this spec")
        guard let c = mjs_addCamera(parent, nil) else {
            preconditionFailure("addCamera: mjs_addCamera returned NULL")
        }
        _ = mjs_setName(c.pointee.element, name)
        c.pointee.pos = (pos[0], pos[1], pos[2])
        c.pointee.fovy = fovy
    }

    /// Every body name in this spec, world first.
    ///
    /// Implemented via compile-and-inspect rather than walking `mjsBody`
    /// children directly: MuJoCo's spec tree-walking API (`mjs_nextChild`)
    /// takes an `mjsBody*` as its first argument, not an `mjsElement*`, which
    /// does not compose cleanly here without extra unwrapping across 3.x
    /// minors. Compiling a throwaway copy is correct and adequate for what
    /// callers need this for (name lookups before/after attach).
    ///
    /// Throws rather than returning `[]` on a compile failure: an empty array and
    /// "this spec does not compile" are very different answers, and swallowing
    /// the latter turned a broken `attach` into a silently empty name list.
    public func findBodyNames() throws -> [String] {
        try compileCopy().bodyNames
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
        // Grow the buffer instead of failing at a fixed 1 MiB: a model with any
        // real mesh or keyframe content serialises past that, and the old code
        // surfaced it as an opaque "buffer too small" error the caller could do
        // nothing about. Cap the growth so a pathological model can't OOM us.
        var capacity = 1 << 20
        let maxCapacity = 1 << 27          // 128 MiB of MJCF is not a real model
        while true {
            var buf = [CChar](repeating: 0, count: capacity)
            var err = [CChar](repeating: 0, count: 1000)
            if mj_saveXMLString(copy, &buf, Int32(buf.count), &err, Int32(err.count)) >= 0 {
                return buf.withUnsafeBufferPointer { String(cString: $0.baseAddress!) }
            }
            let message = err.withUnsafeBufferPointer { String(cString: $0.baseAddress!) }
            // MuJoCo reports an undersized destination via this message; anything
            // else is a real failure and must not be retried.
            guard message.lowercased().contains("too small"), capacity < maxCapacity else {
                throw MjError(message)
            }
            capacity *= 4
        }
    }

    /// Maps a wrapper geom type to MuJoCo's. `.other` is rejected by `addGeom`
    /// before reaching here, so it is unreachable rather than silently a box.
    private func cGeomType(_ t: MjModel.GeomType) -> mjtGeom {
        switch t {
        case .plane: return mjGEOM_PLANE
        case .sphere: return mjGEOM_SPHERE
        case .capsule: return mjGEOM_CAPSULE
        case .ellipsoid: return mjGEOM_ELLIPSOID
        case .cylinder: return mjGEOM_CYLINDER
        case .box: return mjGEOM_BOX
        case .mesh: return mjGEOM_MESH
        case .other:
            preconditionFailure("cGeomType: .other has no MuJoCo geom type; addGeom rejects it")
        }
    }
}

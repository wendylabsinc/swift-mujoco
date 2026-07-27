import CMuJoCo
import Foundation

public let objJoint = mjOBJ_JOINT
public let objActuator = mjOBJ_ACTUATOR
public let objBody = mjOBJ_BODY
public let objGeom = mjOBJ_GEOM
public let objSensor = mjOBJ_SENSOR
public let objMesh = mjOBJ_MESH
public let objKey = mjOBJ_KEY
public let objCamera = mjOBJ_CAMERA
public let objSite = mjOBJ_SITE

/// Intentionally NOT `Sendable`: this class wraps mutable MuJoCo C state
/// (`mjModel`) and must stay on a single isolation domain — stepping and
/// other MuJoCo APIs mutate shared state that is not safe to touch
/// concurrently from multiple isolation domains.
public final class MjModel {
    /// Deliberate low-level escape hatch for downstream C interop with the
    /// raw `mjModel`. The library retains ownership of this pointer (it is
    /// freed in `deinit`); callers must NOT free it or hand it to another
    /// wrapper that assumes ownership.
    ///
    /// - Warning: `joints`, `actuators`, `sensors` and `bodyNames` are cached
    ///   on first access (see below). If you mutate model fields through
    ///   `ptr` after reading one of those — e.g. writing `jnt_range` or
    ///   `actuator_ctrlrange` in place for runtime domain randomisation — the
    ///   cached `JointInfo.range` / `ActuatorInfo.ctrlRange` will silently go
    ///   stale; there is no invalidation. Either avoid those four accessors
    ///   after mutating through `ptr`, or construct a fresh `MjModel` instead.
    public let ptr: UnsafeMutablePointer<mjModel>

    // Introspection is a load-time snapshot: MuJoCo's model topology cannot
    // change without a recompile, and recomputing these O(n) lists on every
    // access showed up as avoidable work in a 200 Hz loop.
    //
    // This assumes topology is immutable after compilation, which holds for
    // MuJoCo's own API — but NOT for a caller writing through the public
    // `ptr` escape hatch above. Poking `ptr.pointee.jnt_range` or
    // `actuator_ctrlrange` in place (a common domain-randomisation pattern)
    // after `joints`/`actuators`/`sensors`/`bodyNames` have already been read
    // will not be reflected; the cache is never invalidated.
    //
    // `internal`, not `private`, so `@testable import` can assert the cache is
    // actually populated — `private` is invisible even to @testable.
    var cachedJoints: [JointInfo]?
    var cachedActuators: [ActuatorInfo]?
    var cachedSensors: [SensorInfo]?
    var cachedBodyNames: [String]?

    init(owning ptr: UnsafeMutablePointer<mjModel>) { self.ptr = ptr }
    deinit { mj_deleteModel(ptr) }

    public static func load(xmlPath: String) throws -> MjModel {
        var err = [CChar](repeating: 0, count: 1000)
        let m = mj_loadXML(xmlPath, nil, &err, Int32(err.count))
        guard let m else {
            throw MjError(err.withUnsafeBufferPointer { String(cString: $0.baseAddress!) })
        }
        return MjModel(owning: m)
    }

    public static func load(xml: String) throws -> MjModel {
        let dir = FileManager.default.temporaryDirectory
        let file = dir.appendingPathComponent("mj-\(UUID().uuidString).xml")
        try xml.write(to: file, atomically: true, encoding: .utf8)
        defer { try? FileManager.default.removeItem(at: file) }
        return try load(xmlPath: file.path)
    }

    public var ngeom: Int { Int(ptr.pointee.ngeom) }
    public var nq: Int { Int(ptr.pointee.nq) }
    public var nv: Int { Int(ptr.pointee.nv) }
    public var nu: Int { Int(ptr.pointee.nu) }
    public var nbody: Int { Int(ptr.pointee.nbody) }
    public var njnt: Int { Int(ptr.pointee.njnt) }
    public var nsensor: Int { Int(ptr.pointee.nsensor) }
    public var nmesh: Int { Int(ptr.pointee.nmesh) }
    public var nkey: Int { Int(ptr.pointee.nkey) }

    public var timestep: Double { ptr.pointee.opt.timestep }
    public var gravity: (Double, Double, Double) {
        let g = ptr.pointee.opt.gravity
        return (g.0, g.1, g.2)   // mjtNum[3] imports as a Swift tuple
    }

    public func name(of obj: mjtObj, id: Int) -> String? {
        guard let c = mj_id2name(ptr, Int32(obj.rawValue), Int32(id)) else { return nil }
        return String(cString: c)
    }

    public func id(of obj: mjtObj, name: String) -> Int? {
        let i = Int(mj_name2id(ptr, Int32(obj.rawValue), name))
        return i >= 0 ? i : nil
    }

    public struct JointInfo {
        public let id: Int
        public let name: String
        public let type: Int
        public let limited: Bool
        public let range: (Double, Double)
        public let qposadr: Int
        public let dofadr: Int
    }

    public struct ActuatorInfo {
        public let id: Int
        public let name: String
        public let ctrlLimited: Bool
        public let ctrlRange: (Double, Double)
    }

    public struct SensorInfo {
        public let id: Int
        public let name: String
        public let type: Int
        public let dim: Int
        public let adr: Int
    }

    public var joints: [JointInfo] {
        if let cachedJoints { return cachedJoints }
        let v = computeJoints()
        cachedJoints = v
        return v
    }

    private func computeJoints() -> [JointInfo] {
        var result: [JointInfo] = []
        for j in 0..<njnt {
            let nm = name(of: objJoint, id: j) ?? ""
            let tp = Int(ptr.pointee.jnt_type[j])
            let lim = ptr.pointee.jnt_limited[j]
            let rng = (ptr.pointee.jnt_range[j*2+0], ptr.pointee.jnt_range[j*2+1])
            let qpa = Int(ptr.pointee.jnt_qposadr[j])
            let dfa = Int(ptr.pointee.jnt_dofadr[j])
            result.append(JointInfo(id: j, name: nm, type: tp, limited: lim, range: rng, qposadr: qpa, dofadr: dfa))
        }
        return result
    }

    public var actuators: [ActuatorInfo] {
        if let cachedActuators { return cachedActuators }
        let v = computeActuators()
        cachedActuators = v
        return v
    }

    private func computeActuators() -> [ActuatorInfo] {
        var result: [ActuatorInfo] = []
        for a in 0..<nu {
            let nm = name(of: objActuator, id: a) ?? ""
            let ctl = ptr.pointee.actuator_ctrllimited[a]
            let rng = (ptr.pointee.actuator_ctrlrange[a*2+0], ptr.pointee.actuator_ctrlrange[a*2+1])
            result.append(ActuatorInfo(id: a, name: nm, ctrlLimited: ctl, ctrlRange: rng))
        }
        return result
    }

    public var sensors: [SensorInfo] {
        if let cachedSensors { return cachedSensors }
        let v = computeSensors()
        cachedSensors = v
        return v
    }

    private func computeSensors() -> [SensorInfo] {
        var result: [SensorInfo] = []
        for s in 0..<nsensor {
            let nm = name(of: objSensor, id: s) ?? ""
            let tp = Int(ptr.pointee.sensor_type[s])
            let dm = Int(ptr.pointee.sensor_dim[s])
            let adr = Int(ptr.pointee.sensor_adr[s])
            result.append(SensorInfo(id: s, name: nm, type: tp, dim: dm, adr: adr))
        }
        return result
    }

    public var bodyNames: [String] {
        if let cachedBodyNames { return cachedBodyNames }
        let v = computeBodyNames()
        cachedBodyNames = v
        return v
    }

    private func computeBodyNames() -> [String] {
        (0..<nbody).map { name(of: objBody, id: $0) ?? "" }
    }

    public enum GeomType: String {
        case plane, sphere, capsule, ellipsoid, cylinder, box, mesh, other
    }

    public func geomType(_ i: Int) -> GeomType {
        precondition(i >= 0 && i < ngeom)
        switch Int32(ptr.pointee.geom_type[i]) {
        case Int32(mjGEOM_PLANE.rawValue): return .plane
        case Int32(mjGEOM_SPHERE.rawValue): return .sphere
        case Int32(mjGEOM_CAPSULE.rawValue): return .capsule
        case Int32(mjGEOM_ELLIPSOID.rawValue): return .ellipsoid
        case Int32(mjGEOM_CYLINDER.rawValue): return .cylinder
        case Int32(mjGEOM_BOX.rawValue): return .box
        case Int32(mjGEOM_MESH.rawValue): return .mesh
        default: return .other
        }
    }

    public func geomSize(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < ngeom)
        return [ptr.pointee.geom_size[i * 3 + 0],
         ptr.pointee.geom_size[i * 3 + 1],
         ptr.pointee.geom_size[i * 3 + 2]]
    }

    public func geomGroup(_ i: Int) -> Int {
        precondition(i >= 0 && i < ngeom)
        return Int(ptr.pointee.geom_group[i])
    }
    public func geomDataid(_ i: Int) -> Int {
        precondition(i >= 0 && i < ngeom)
        return Int(ptr.pointee.geom_dataid[i])
    }

    public func geomRgba(_ i: Int) -> [Double] {
        precondition(i >= 0 && i < ngeom)
        let matid = Int(ptr.pointee.geom_matid[i])
        let base: UnsafeMutablePointer<Float>
        let off: Int
        if matid >= 0 { base = ptr.pointee.mat_rgba; off = matid * 4 }
        else { base = ptr.pointee.geom_rgba; off = i * 4 }
        return (0..<4).map { Double(base[off + $0]) }
    }

    public func geomIsVisible(_ i: Int) -> Bool {
        geomGroup(i) < 3 && geomRgba(i)[3] != 0.0
    }

    public func meshVertices(_ meshId: Int) -> [Float] {
        precondition(meshId >= 0 && meshId < nmesh)
        let v0 = Int(ptr.pointee.mesh_vertadr[meshId])
        let vn = Int(ptr.pointee.mesh_vertnum[meshId])
        guard let base = ptr.pointee.mesh_vert else { return [] }
        return (0..<(vn * 3)).map { base[(v0 * 3) + $0] }
    }

    public func meshFaces(_ meshId: Int) -> [Int] {
        precondition(meshId >= 0 && meshId < nmesh)
        let f0 = Int(ptr.pointee.mesh_faceadr[meshId])
        let fn = Int(ptr.pointee.mesh_facenum[meshId])
        guard let base = ptr.pointee.mesh_face else { return [] }
        return (0..<(fn * 3)).map { Int(base[(f0 * 3) + $0]) }
    }

    public func meshName(_ meshId: Int) -> String {
        precondition(meshId >= 0 && meshId < nmesh)
        return name(of: objMesh, id: meshId) ?? "mesh\(meshId)"
    }
}

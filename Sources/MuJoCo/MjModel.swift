import CMuJoCo
import Foundation

public final class MjModel {
    public let ptr: UnsafeMutablePointer<mjModel>

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

    public enum GeomType: String {
        case plane, sphere, capsule, ellipsoid, cylinder, box, mesh, other
    }

    public func geomType(_ i: Int) -> GeomType {
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
        [ptr.pointee.geom_size[i * 3 + 0],
         ptr.pointee.geom_size[i * 3 + 1],
         ptr.pointee.geom_size[i * 3 + 2]]
    }

    public func geomGroup(_ i: Int) -> Int { Int(ptr.pointee.geom_group[i]) }
    public func geomDataid(_ i: Int) -> Int { Int(ptr.pointee.geom_dataid[i]) }

    public func geomRgba(_ i: Int) -> [Double] {
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
        let v0 = Int(ptr.pointee.mesh_vertadr[meshId])
        let vn = Int(ptr.pointee.mesh_vertnum[meshId])
        guard let base = ptr.pointee.mesh_vert else { return [] }
        return (0..<(vn * 3)).map { base[(v0 * 3) + $0] }
    }

    public func meshFaces(_ meshId: Int) -> [Int] {
        let f0 = Int(ptr.pointee.mesh_faceadr[meshId])
        let fn = Int(ptr.pointee.mesh_facenum[meshId])
        guard let base = ptr.pointee.mesh_face else { return [] }
        return (0..<(fn * 3)).map { Int(base[(f0 * 3) + $0]) }
    }

    private func _tmpName(_ obj: mjtObj, _ id: Int) -> String? {
        guard let c = mj_id2name(ptr, Int32(obj.rawValue), Int32(id)) else { return nil }
        return String(cString: c)
    }

    public func meshName(_ meshId: Int) -> String {
        _tmpName(mjOBJ_MESH, meshId) ?? "mesh\(meshId)"
    }
}

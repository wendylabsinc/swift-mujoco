import CMuJoCo
import Foundation

public final class MjModel {
    public let ptr: UnsafeMutablePointer<mjModel>

    init(owning ptr: UnsafeMutablePointer<mjModel>) { self.ptr = ptr }
    deinit { mj_deleteModel(ptr) }

    public static func load(xmlPath: String) throws -> MjModel {
        var err = [CChar](repeating: 0, count: 1000)
        let m = mj_loadXML(xmlPath, nil, &err, Int32(err.count))
        guard let m else { throw MjError(String(cString: err)) }
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
}

import MuJoCo

public struct Geom: Encodable {
    public let i: Int
    public let type: String
    public let size: [Double]
    public let rgba: [Double]
    public let mesh: String?   // synthesized Encodable omits this key when nil
}

public struct MeshBuf: Encodable {
    public let vert: [Double]
    public let face: [Int]
}

public struct SceneManifest: Encodable {
    public let title: String
    public let up: String
    public let engine: String
    public let geoms: [Geom]
    public let meshes: [String: MeshBuf]
}

/// mjModel -> one-time scene manifest (visible geoms + deduplicated mesh buffers).
public func buildScene(_ model: MjModel, title: String) -> SceneManifest {
    var geoms: [Geom] = []
    var meshes: [String: MeshBuf] = [:]
    for i in 0..<model.ngeom where model.geomIsVisible(i) {
        let kind = model.geomType(i).rawValue
        var meshName: String? = nil
        if model.geomType(i) == .mesh {
            let mid = model.geomDataid(i)
            let name = model.meshName(mid)
            meshName = name
            if meshes[name] == nil {
                meshes[name] = MeshBuf(
                    vert: model.meshVertices(mid).map { mjRound(Double($0), 4) },
                    face: model.meshFaces(mid))
            }
        }
        geoms.append(Geom(
            i: i,
            type: kind,
            size: model.geomSize(i).map { mjRound($0, 5) },
            rgba: model.geomRgba(i).map { mjRound($0, 4) },
            mesh: meshName))
    }
    return SceneManifest(title: title, up: "z", engine: "mujoco", geoms: geoms, meshes: meshes)
}

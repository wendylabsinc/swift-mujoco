import CMuJoCo

/// A geom (visual/collision shape) attached to its parent body.
public struct Geom: MjSceneElement {
    public let name: String?
    public let type: MjModel.GeomType
    public let size: [Double]
    public let pos: [Double]
    public let rgba: [Double]

    public init(name: String? = nil, type: MjModel.GeomType, size: [Double],
                pos: [Double] = [0, 0, 0], rgba: [Double] = [0.5, 0.5, 0.5, 1]) {
        self.name = name
        self.type = type
        self.size = size
        self.pos = pos
        self.rgba = rgba
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        let g = mjs_addGeom(parent.ptr, nil)!
        if let name { _ = mjs_setName(g.pointee.element, name) }
        g.pointee.type = spec.cGeomType(type)
        g.pointee.size = (size[0], size[1], size[2])
        g.pointee.pos = (pos[0], pos[1], pos[2])
        g.pointee.rgba = (Float(rgba[0]), Float(rgba[1]), Float(rgba[2]), Float(rgba[3]))
    }
}

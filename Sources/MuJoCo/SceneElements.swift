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
        guard let g = mjs_addGeom(parent.ptr, nil) else {
            preconditionFailure("Geom.apply: mjs_addGeom returned NULL")
        }
        if let name { _ = mjs_setName(g.pointee.element, name) }
        spec.configureGeom(g, type: type, size: size, pos: pos, rgba: rgba)
    }
}

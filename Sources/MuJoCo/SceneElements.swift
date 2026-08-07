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

/// A body, optionally carrying its own children (joints, geoms, nested
/// bodies, sites, cameras, lights).
public struct Body: MjSceneElement {
    public let name: String?
    public let pos: [Double]
    public let quat: [Double]?
    public let children: [MjSceneElement]

    public init(name: String? = nil, pos: [Double] = [0, 0, 0], quat: [Double]? = nil,
                @MjSceneBuilder children: () -> [MjSceneElement] = { [] }) {
        self.name = name
        self.pos = pos
        self.quat = quat
        self.children = children()
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        precondition(pos.count == 3, "Body: pos needs 3 components, got \(pos.count)")
        if let quat {
            precondition(quat.count == 4, "Body: quat needs 4 components, got \(quat.count)")
        }
        guard let b = mjs_addBody(parent.ptr, nil) else {
            preconditionFailure("Body.apply: mjs_addBody returned NULL")
        }
        if let name { _ = mjs_setName(b.pointee.element, name) }
        b.pointee.pos = (pos[0], pos[1], pos[2])
        if let quat {
            b.pointee.quat = (quat[0], quat[1], quat[2], quat[3])
        }
        let bodyHandle = MjSpecBody(ptr: b)
        for child in children {
            child.apply(spec: spec, parent: bodyHandle)
        }
    }
}

/// A free (6-DOF) joint — global position and orientation, unconstrained.
public struct FreeJoint: MjSceneElement {
    public let name: String?

    public init(name: String? = nil) {
        self.name = name
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        guard let j = mjs_addFreeJoint(parent.ptr) else {
            preconditionFailure("FreeJoint.apply: mjs_addFreeJoint returned NULL")
        }
        if let name { _ = mjs_setName(j.pointee.element, name) }
    }
}

/// The joint kinds reachable through `Joint`. Free joints have their own
/// dedicated `FreeJoint` element, so `mjJNT_FREE` is never produced here.
public enum JointKind {
    case hinge, slide, ball
}

private func cJointType(_ t: JointKind) -> mjtJoint {
    switch t {
    case .hinge: return mjJNT_HINGE
    case .slide: return mjJNT_SLIDE
    case .ball: return mjJNT_BALL
    }
}

/// A hinge, slide, or ball joint. `range`, when non-nil, sets both the
/// joint's limits and marks it as limited; when nil, the joint is
/// unlimited (MuJoCo's own default for a freshly added joint).
///
/// The `range` parameter is specified in **degrees**, following MJCF's default
/// `compiler angle="degree"` convention; MuJoCo's compiler automatically converts
/// it to radians during model compilation.
public struct Joint: MjSceneElement {
    public let name: String?
    public let type: JointKind
    public let axis: [Double]
    public let pos: [Double]
    public let range: ClosedRange<Double>?

    public init(name: String? = nil, type: JointKind, axis: [Double] = [0, 0, 1],
                pos: [Double] = [0, 0, 0], range: ClosedRange<Double>? = nil) {
        self.name = name
        self.type = type
        self.axis = axis
        self.pos = pos
        self.range = range
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        precondition(axis.count == 3, "Joint: axis needs 3 components, got \(axis.count)")
        precondition(pos.count == 3, "Joint: pos needs 3 components, got \(pos.count)")
        guard let j = mjs_addJoint(parent.ptr, nil) else {
            preconditionFailure("Joint.apply: mjs_addJoint returned NULL")
        }
        if let name { _ = mjs_setName(j.pointee.element, name) }
        j.pointee.type = cJointType(type)
        j.pointee.axis = (axis[0], axis[1], axis[2])
        j.pointee.pos = (pos[0], pos[1], pos[2])
        if let range {
            j.pointee.range = (range.lowerBound, range.upperBound)
            j.pointee.limited = 1
        }
    }
}

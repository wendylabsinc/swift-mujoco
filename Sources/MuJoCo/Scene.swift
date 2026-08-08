import CMuJoCo

/// The root of a scene DSL tree — a SwiftUI-like alternative to writing raw
/// MJCF XML by hand. Builds a **bare** `MjSpec` (no default floor/light,
/// unlike `MjSpec`'s own convenience initializer, which would otherwise
/// silently duplicate whatever the scene itself declares) and applies every
/// top-level element against the world body.
public final class Scene {
    private let elements: [MjSceneElement]

    public init(@MjSceneBuilder _ content: () -> [MjSceneElement]) {
        self.elements = content()
    }

    /// The live, still-editable spec this scene builds — e.g. for a caller
    /// that wants to `attach()` more content onto it afterwards.
    public func spec() -> MjSpec {
        let spec = MjSpec(floor: false, light: false)
        // Every `MjSpec` always has a "world" body — `mj_makeSpec` creates it
        // unconditionally — so this cannot fail on a freshly built bare spec.
        guard let worldPtr = mjs_findBody(spec.ptr, "world") else {
            preconditionFailure("a fresh mjSpec has no \"world\" body; MuJoCo version mismatch?")
        }
        let world = MjSpecBody(ptr: worldPtr)
        for element in elements {
            element.apply(spec: spec, parent: world)
        }
        return spec
    }

    public func compile() throws -> MjModel { try spec().compile() }
    public func xml() throws -> String { try spec().saveXML() }
}

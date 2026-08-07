import CMuJoCo

/// Opaque handle to a body inside a scene under construction. Produced by
/// `Scene`'s root and by `Body.apply`. The wrapped pointer is `internal`, so
/// this type carries no unsafe API across a module boundary — a caller
/// outside `MuJoCo` can hold one, pass it along, but never construct,
/// inspect, or dereference it.
public struct MjSpecBody {
    let ptr: UnsafeMutablePointer<mjsBody>
}

/// One node in a `Scene`'s declarative tree — a body, geom, joint, or other
/// MJCF element. Conforming types apply themselves onto the `MjSpec` being
/// built; `Scene` walks the tree once, in order, to compose the whole model.
public protocol MjSceneElement {
    func apply(spec: MjSpec, parent: MjSpecBody)
}

/// Combines `MjSceneElement`s inside a `Scene`'s or `Body`'s trailing
/// closure — mirrors `ViewBuilder` closely enough that `if`/`for` work the
/// same way they do in SwiftUI. Simpler than `ViewBuilder` itself: elements
/// are heterogeneous protocol existentials collected into a plain
/// `[MjSceneElement]` array, since nothing here re-renders — it is walked
/// exactly once.
@resultBuilder
public struct MjSceneBuilder {
    public static func buildBlock(_ components: [MjSceneElement]...) -> [MjSceneElement] {
        components.flatMap { $0 }
    }

    public static func buildOptional(_ component: [MjSceneElement]?) -> [MjSceneElement] {
        component ?? []
    }

    public static func buildEither(first component: [MjSceneElement]) -> [MjSceneElement] {
        component
    }

    public static func buildEither(second component: [MjSceneElement]) -> [MjSceneElement] {
        component
    }

    public static func buildArray(_ components: [[MjSceneElement]]) -> [MjSceneElement] {
        components.flatMap { $0 }
    }

    public static func buildExpression(_ expression: MjSceneElement) -> [MjSceneElement] {
        [expression]
    }
}

# MuJoCo Scene DSL Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A SwiftUI-like result-builder DSL (`Scene { Geom(...); Body(...) { ... } }`) for building MuJoCo models in Swift, as an ergonomic alternative to hand-written MJCF XML strings.

**Architecture:** A `MjSceneElement` protocol (`apply(spec: MjSpec, parent: MjSpecBody)`) drives composition; concrete elements (`Body`, `Geom`, `Joint`, `FreeJoint`, `Site`, `Camera`, `Light`, `Option`) call the matching `mjs_add*` C function on their parent and recurse. `MjSpecBody` hides the underlying `UnsafeMutablePointer<mjsBody>` behind an opaque public struct so no `Unsafe*` type appears in public API. A `@resultBuilder MjSceneBuilder` combines elements the way `ViewBuilder` does, and `Scene` is the root that turns a tree into an `MjSpec`/`MjModel`/XML string.

**Tech Stack:** Swift 6.1, MuJoCo's `mjSpec` C API (via the existing `CMuJoCo` system-library target), Swift Testing (`@Test`/`#expect`).

## Global Constraints

- New code lives inside the existing `MuJoCo` SPM target — no new target or product, no `Package.swift` changes.
- No `Unsafe*` type may appear in the public signature of `MjSceneElement`, `Scene`, or any element type. Raw pointers are hidden behind `MjSpecBody` (public struct, `internal` stored pointer).
- Reuse `MjModel.GeomType` for geom types — do not introduce a second geom-type enum.
- `pos`/`size`/`rgba`/`axis` parameters are plain `[Double]`, matching `MjSpec.addGeom`/`addSite`/`addCamera`'s existing convention — no new vector type.
- `mjs_add*` calls are force-unwrapped (`!`), matching the existing pattern throughout `MjSpec.swift` — they fail only on memory exhaustion, not on ordinary input.
- Every new/modified file lives in `Sources/MuJoCo/` or `Tests/MuJoCoTests/`, both already-existing directories.

---

### Task 1: Core protocol, opaque handle, result builder, `Scene` root, and `Geom`

**Files:**
- Modify: `Sources/MuJoCo/MjSpec.swift` — relax `cGeomType` from `private` to `internal` (drop the `private` keyword) so the new `Geom` element can reuse the existing `MjModel.GeomType` → `mjtGeom` mapping instead of duplicating it.
- Create: `Sources/MuJoCo/SceneElement.swift`
- Create: `Sources/MuJoCo/Scene.swift`
- Create: `Sources/MuJoCo/SceneElements.swift`
- Test: `Tests/MuJoCoTests/SceneDSLTests.swift`

**Interfaces:**
- Consumes: `MjSpec` (`Sources/MuJoCo/MjSpec.swift`) — `public let ptr: UnsafeMutablePointer<mjSpec>`, `public init(floor: Bool = true, light: Bool = true)`, `func cGeomType(_ t: MjModel.GeomType) -> mjtGeom` (after this task's visibility change), `public func compile() throws -> MjModel`, `public func saveXML() throws -> String`. `MjModel.GeomType` (`Sources/MuJoCo/MjModel.swift:191`) — `public enum GeomType: String, Sendable { case plane, sphere, capsule, ellipsoid, cylinder, box, mesh, other }`.
- Produces: `public struct MjSpecBody { let ptr: UnsafeMutablePointer<mjsBody> }`; `public protocol MjSceneElement { func apply(spec: MjSpec, parent: MjSpecBody) }`; `@resultBuilder public struct MjSceneBuilder`; `public final class Scene { public init(@MjSceneBuilder _ content: () -> [MjSceneElement]); public func spec() -> MjSpec; public func compile() throws -> MjModel; public func xml() throws -> String }`; `public struct Geom: MjSceneElement { public init(name: String? = nil, type: MjModel.GeomType, size: [Double], pos: [Double] = [0,0,0], rgba: [Double] = [0.5,0.5,0.5,1]) }`.

- [ ] **Step 1: Write the failing test**

Create `Tests/MuJoCoTests/SceneDSLTests.swift`:

```swift
import Testing
@testable import MuJoCo

@Test func sceneWithSingleGeomCompiles() throws {
    let scene = Scene {
        Geom(name: "floor", type: .plane, size: [5, 5, 0.1], rgba: [0.2, 0.23, 0.28, 1])
    }
    let model = try scene.compile()

    #expect(model.ngeom == 1)
    #expect(model.geomType(0) == .plane)
    #expect(model.geomSize(0) == [5, 5, 0.1])
    #expect(model.id(of: objGeom, name: "floor") != nil)

    let rgba = model.geomRgba(0)
    let expectedRgba = [0.2, 0.23, 0.28, 1.0]
    for (a, b) in zip(rgba, expectedRgba) {
        #expect(abs(a - b) < 1e-5)
    }
}

@Test func sceneDefaultGeomParametersMatchDocumentedDefaults() throws {
    let scene = Scene {
        Geom(type: .sphere, size: [0.2, 0, 0])
    }
    let model = try scene.compile()

    #expect(model.ngeom == 1)
    let rgba = model.geomRgba(0)
    for (a, b) in zip(rgba, [0.5, 0.5, 0.5, 1.0]) {
        #expect(abs(a - b) < 1e-5)
    }
    #expect(model.geomSize(0) == [0.2, 0, 0])
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `swift test --filter SceneDSLTests`
Expected: FAIL to build — "cannot find 'Scene' in scope", "cannot find 'Geom' in scope".

- [ ] **Step 3: Relax `cGeomType`'s access level**

In `Sources/MuJoCo/MjSpec.swift`, change:

```swift
    private func cGeomType(_ t: MjModel.GeomType) -> mjtGeom {
```

to:

```swift
    func cGeomType(_ t: MjModel.GeomType) -> mjtGeom {
```

(No other change to that method's body.)

- [ ] **Step 4: Create the protocol, opaque handle, and result builder**

Create `Sources/MuJoCo/SceneElement.swift`:

```swift
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
```

- [ ] **Step 5: Create `Scene`**

Create `Sources/MuJoCo/Scene.swift`:

```swift
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
        let world = MjSpecBody(ptr: mjs_findBody(spec.ptr, "world")!)
        for element in elements {
            element.apply(spec: spec, parent: world)
        }
        return spec
    }

    public func compile() throws -> MjModel { try spec().compile() }
    public func xml() throws -> String { try spec().saveXML() }
}
```

- [ ] **Step 6: Create `Geom`**

Create `Sources/MuJoCo/SceneElements.swift`:

```swift
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
```

- [ ] **Step 7: Run the tests to verify they pass**

Run: `swift test --filter SceneDSLTests`
Expected: PASS (both tests).

- [ ] **Step 8: Commit**

```bash
git add Sources/MuJoCo/MjSpec.swift Sources/MuJoCo/SceneElement.swift \
        Sources/MuJoCo/Scene.swift Sources/MuJoCo/SceneElements.swift \
        Tests/MuJoCoTests/SceneDSLTests.swift
git commit -m "feat: add Scene/Geom scene DSL core (MjSceneElement, MjSpecBody, MjSceneBuilder)"
```

---

### Task 2: `Body` (nesting) and `FreeJoint`

**Files:**
- Modify: `Sources/MuJoCo/SceneElements.swift` — add `Body` and `FreeJoint`.
- Modify: `Tests/MuJoCoTests/SceneDSLTests.swift` — add nesting tests.

**Interfaces:**
- Consumes (from Task 1): `MjSceneElement`, `MjSpecBody(ptr:)` (internal init, same module), `MjSceneBuilder`, `Scene`, `Geom`. `MjModel.id(of:name:)`, `objBody`/`objGeom` (`Sources/MuJoCo/MjModel.swift`). `MjData(_:)` init, `mjForward(_:_:)`, `MjData.bodyPos(_:) -> Vec3` (`Sources/MuJoCo/MjBodies.swift`). `Vec3(_:_:_:)`, `Equatable`.
- Produces: `public struct Body: MjSceneElement { public init(name: String? = nil, pos: [Double] = [0,0,0], quat: [Double]? = nil, @MjSceneBuilder children: () -> [MjSceneElement] = { }) }`; `public struct FreeJoint: MjSceneElement { public init(name: String? = nil) }`.

- [ ] **Step 1: Write the failing test**

Append to `Tests/MuJoCoTests/SceneDSLTests.swift`:

```swift
@Test func nestedBodyWithFreeJointAndGeom() throws {
    let scene = Scene {
        Body(name: "cube", pos: [0, 0, 2]) {
            FreeJoint()
            Geom(name: "box", type: .box, size: [0.15, 0.15, 0.15], rgba: [0.2, 0.6, 0.9, 1])
        }
    }
    let model = try scene.compile()

    #expect(model.nbody == 2)   // world + cube
    #expect(model.njnt == 1)
    let cube = try #require(model.id(of: objBody, name: "cube"))
    #expect(model.id(of: objGeom, name: "box") != nil)

    let data = MjData(model)
    mjForward(model, data)
    let p = data.bodyPos(cube)
    #expect(abs(p.x) < 1e-9 && abs(p.y) < 1e-9 && abs(p.z - 2) < 1e-9)
}

@Test func bodyWithNoTrailingClosureHasNoChildren() throws {
    // Exercises Body's default `children: () -> [MjSceneElement] = { }` —
    // a childless body (e.g. a bare attachment point).
    let scene = Scene {
        Geom(type: .plane, size: [5, 5, 0.1])
        Body(name: "anchor", pos: [1, 1, 0])
    }
    let model = try scene.compile()
    #expect(model.nbody == 2)   // world + anchor
    #expect(model.ngeom == 1)   // only the floor — anchor has no geom of its own
    #expect(model.id(of: objBody, name: "anchor") != nil)
}

@Test func twoLevelsOfNestedBodies() throws {
    let scene = Scene {
        Body(name: "outer", pos: [1, 0, 0]) {
            Body(name: "inner", pos: [0, 0, 1]) {
                Geom(type: .sphere, size: [0.1, 0, 0])
            }
        }
    }
    let model = try scene.compile()

    #expect(model.nbody == 3)   // world + outer + inner
    let inner = try #require(model.id(of: objBody, name: "inner"))
    let data = MjData(model)
    mjForward(model, data)
    let p = data.bodyPos(inner)
    // inner's local pos (0,0,1) composed with outer's world pos (1,0,0).
    #expect(abs(p.x - 1) < 1e-9 && abs(p.y) < 1e-9 && abs(p.z - 1) < 1e-9)
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `swift test --filter SceneDSLTests`
Expected: FAIL to build — "cannot find 'Body' in scope", "cannot find 'FreeJoint' in scope".

- [ ] **Step 3: Implement `Body` and `FreeJoint`**

Append to `Sources/MuJoCo/SceneElements.swift`:

```swift
/// A body, optionally carrying its own children (joints, geoms, nested
/// bodies, sites, cameras, lights).
public struct Body: MjSceneElement {
    public let name: String?
    public let pos: [Double]
    public let quat: [Double]?
    public let children: [MjSceneElement]

    public init(name: String? = nil, pos: [Double] = [0, 0, 0], quat: [Double]? = nil,
                @MjSceneBuilder children: () -> [MjSceneElement] = { }) {
        self.name = name
        self.pos = pos
        self.quat = quat
        self.children = children()
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        let b = mjs_addBody(parent.ptr, nil)!
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
        let j = mjs_addFreeJoint(parent.ptr)!
        if let name { _ = mjs_setName(j.pointee.element, name) }
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `swift test --filter SceneDSLTests`
Expected: PASS (all five tests so far).

- [ ] **Step 5: Commit**

```bash
git add Sources/MuJoCo/SceneElements.swift Tests/MuJoCoTests/SceneDSLTests.swift
git commit -m "feat: add Body and FreeJoint scene DSL elements"
```

---

### Task 3: `Joint` (hinge/slide/ball)

**Files:**
- Modify: `Sources/MuJoCo/SceneElements.swift` — add `JointKind` and `Joint`.
- Modify: `Tests/MuJoCoTests/SceneDSLTests.swift` — add a hinge-joint-with-range test.

**Interfaces:**
- Consumes (from Tasks 1–2): `MjSceneElement`, `MjSpecBody`, `Scene`, `Body`, `Geom`. `MjModel.joints: [JointInfo]` and `JointInfo { let id: Int; let name: String; let type: Int; let limited: Bool; let range: (Double, Double); ... }` (`Sources/MuJoCo/MjModel.swift:97-141`).
- Produces: `public enum JointKind { case hinge, slide, ball }`; `public struct Joint: MjSceneElement { public init(name: String? = nil, type: JointKind, axis: [Double] = [0,0,1], pos: [Double] = [0,0,0], range: ClosedRange<Double>? = nil) }`.

- [ ] **Step 1: Write the failing test**

Append to `Tests/MuJoCoTests/SceneDSLTests.swift`:

```swift
@Test func hingeJointWithRangeAndAxis() throws {
    let scene = Scene {
        Body(name: "pole", pos: [0, 0, 1]) {
            Joint(name: "hinge", type: .hinge, axis: [0, 1, 0], range: -1.5...1.5)
            Geom(type: .capsule, size: [0.02, 0.5, 0])
        }
    }
    let model = try scene.compile()

    #expect(model.njnt == 1)
    let joint = try #require(model.joints.first { $0.name == "hinge" })
    #expect(joint.type == 3)   // mjJNT_HINGE (mjtJoint_: FREE=0, BALL=1, SLIDE=2, HINGE=3)
    #expect(joint.limited == true)
    #expect(abs(joint.range.0 - (-1.5)) < 1e-9)
    #expect(abs(joint.range.1 - 1.5) < 1e-9)
}

@Test func slideAndBallJointsCompile() throws {
    let scene = Scene {
        Body(name: "a") {
            Joint(type: .slide, axis: [1, 0, 0])
            Geom(type: .box, size: [0.1, 0.1, 0.1])
        }
        Body(name: "b") {
            Joint(type: .ball)
            Geom(type: .sphere, size: [0.1, 0, 0])
        }
    }
    let model = try scene.compile()
    #expect(model.njnt == 2)
    #expect(model.joints[0].type == 2)   // mjJNT_SLIDE
    #expect(model.joints[1].type == 1)   // mjJNT_BALL
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `swift test --filter SceneDSLTests`
Expected: FAIL to build — "cannot find 'Joint' in scope", "cannot find type 'JointKind' in scope".

- [ ] **Step 3: Implement `JointKind` and `Joint`**

Append to `Sources/MuJoCo/SceneElements.swift`:

```swift
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
        let j = mjs_addJoint(parent.ptr, nil)!
        if let name { _ = mjs_setName(j.pointee.element, name) }
        j.pointee.type = cJointType(type)
        j.pointee.axis = (axis[0], axis[1], axis[2])
        j.pointee.pos = (pos[0], pos[1], pos[2])
        if let range {
            j.pointee.limited = 1
            j.pointee.range = (range.lowerBound, range.upperBound)
        }
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `swift test --filter SceneDSLTests`
Expected: PASS (all seven tests so far).

- [ ] **Step 5: Commit**

```bash
git add Sources/MuJoCo/SceneElements.swift Tests/MuJoCoTests/SceneDSLTests.swift
git commit -m "feat: add Joint scene DSL element (hinge/slide/ball)"
```

---

### Task 4: `Site`, `Camera`, `Light`

**Files:**
- Modify: `Sources/MuJoCo/MjBodies.swift` — add `MjModel.nlight`, needed to test `Light`.
- Modify: `Sources/MuJoCo/SceneElements.swift` — add `Site`, `Camera`, `Light`.
- Modify: `Tests/MuJoCoTests/SceneDSLTests.swift` — add tests for all three.

**Interfaces:**
- Consumes (from Tasks 1–2): `MjSceneElement`, `MjSpecBody`, `Scene`, `Body`. `MjModel.nsite`/`ncam` (`Sources/MuJoCo/MjBodies.swift:4-5`), `MjModel.camFovy(_:)` (`Sources/MuJoCo/MjRender.swift:17`), `objSite`/`objCamera` (`Sources/MuJoCo/MjModel.swift`).
- Produces: `MjModel.nlight: Int`; `public struct Site: MjSceneElement { public init(name: String? = nil, pos: [Double] = [0,0,0], size: Double = 0.01) }`; `public struct Camera: MjSceneElement { public init(name: String? = nil, pos: [Double] = [0,0,0], fovy: Double = 45) }`; `public struct Light: MjSceneElement { public init(pos: [Double]? = nil) }`.

- [ ] **Step 1: Write the failing test**

Append to `Tests/MuJoCoTests/SceneDSLTests.swift`:

```swift
@Test func siteCameraAndLightAppearInCompiledModel() throws {
    let scene = Scene {
        Body(name: "head", pos: [0, 0, 1]) {
            Geom(type: .sphere, size: [0.1, 0, 0])
            Site(name: "imu", pos: [0, 0, 0.05])
            Camera(name: "eye", pos: [0.1, 0, 0], fovy: 60)
        }
        Light(pos: [0, 0, 3])
    }
    let model = try scene.compile()

    #expect(model.nsite == 1)
    #expect(model.ncam == 1)
    #expect(model.nlight == 1)
    #expect(model.id(of: objSite, name: "imu") != nil)
    #expect(model.id(of: objCamera, name: "eye") != nil)
    #expect(abs(model.camFovy(0) - 60) < 1e-9)
}

@Test func siteAndCameraUseDocumentedDefaults() throws {
    let scene = Scene {
        Body(name: "b") {
            Geom(type: .box, size: [0.1, 0.1, 0.1])
            Site()
            Camera()
        }
    }
    let model = try scene.compile()
    #expect(model.nsite == 1)
    #expect(model.ncam == 1)
    #expect(abs(model.camFovy(0) - 45) < 1e-9)
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `swift test --filter SceneDSLTests`
Expected: FAIL to build — "cannot find 'Site' in scope", "cannot find 'Camera' in scope", "cannot find 'Light' in scope", "value of type 'MjModel' has no member 'nlight'".

- [ ] **Step 3: Add `MjModel.nlight`**

In `Sources/MuJoCo/MjBodies.swift`, change:

```swift
extension MjModel {
    public var nsite: Int { Int(ptr.pointee.nsite) }
    public var ncam: Int { Int(ptr.pointee.ncam) }
}
```

to:

```swift
extension MjModel {
    public var nsite: Int { Int(ptr.pointee.nsite) }
    public var ncam: Int { Int(ptr.pointee.ncam) }
    public var nlight: Int { Int(ptr.pointee.nlight) }
}
```

- [ ] **Step 4: Implement `Site`, `Camera`, `Light`**

Append to `Sources/MuJoCo/SceneElements.swift`:

```swift
/// A site — where IMUs, rangefinders, and touch sensors mount.
public struct Site: MjSceneElement {
    public let name: String?
    public let pos: [Double]
    public let size: Double

    public init(name: String? = nil, pos: [Double] = [0, 0, 0], size: Double = 0.01) {
        self.name = name
        self.pos = pos
        self.size = size
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        let s = mjs_addSite(parent.ptr, nil)!
        if let name { _ = mjs_setName(s.pointee.element, name) }
        s.pointee.pos = (pos[0], pos[1], pos[2])
        s.pointee.size = (size, size, size)
    }
}

/// A fixed camera.
public struct Camera: MjSceneElement {
    public let name: String?
    public let pos: [Double]
    public let fovy: Double

    public init(name: String? = nil, pos: [Double] = [0, 0, 0], fovy: Double = 45) {
        self.name = name
        self.pos = pos
        self.fovy = fovy
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        let c = mjs_addCamera(parent.ptr, nil)!
        if let name { _ = mjs_setName(c.pointee.element, name) }
        c.pointee.pos = (pos[0], pos[1], pos[2])
        c.pointee.fovy = fovy
    }
}

/// A light. `MjSpec`'s own initializer already adds a default light to a
/// scene built with `light: true`; `Scene` always builds with `light: false`
/// (see `Scene.spec()`), so use this element for any light a scene needs.
public struct Light: MjSceneElement {
    public let pos: [Double]?

    public init(pos: [Double]? = nil) {
        self.pos = pos
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        let l = mjs_addLight(parent.ptr, nil)!
        if let pos {
            l.pointee.pos = (pos[0], pos[1], pos[2])
        }
    }
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `swift test --filter SceneDSLTests`
Expected: PASS (all nine tests so far).

- [ ] **Step 6: Commit**

```bash
git add Sources/MuJoCo/MjBodies.swift Sources/MuJoCo/SceneElements.swift \
        Tests/MuJoCoTests/SceneDSLTests.swift
git commit -m "feat: add Site, Camera, and Light scene DSL elements"
```

---

### Task 5: `Option` and control-flow (`if`/`for`) coverage

**Files:**
- Modify: `Sources/MuJoCo/SceneElements.swift` — add `Option`.
- Modify: `Tests/MuJoCoTests/SceneDSLTests.swift` — add `Option` test and control-flow tests.

**Interfaces:**
- Consumes (from Task 1): `MjSceneElement`, `MjSpecBody`, `MjSceneBuilder` (specifically `buildOptional`/`buildArray`, defined but not yet exercised by any test), `Scene`. `MjModel.timestep: Double` (`Sources/MuJoCo/MjModel.swift:81`).
- Produces: `public struct Option: MjSceneElement { public init(timestep: Double) }`.

- [ ] **Step 1: Write the failing test**

Append to `Tests/MuJoCoTests/SceneDSLTests.swift`:

```swift
@Test func optionSetsTimestep() throws {
    let scene = Scene {
        Option(timestep: 0.002)
        Geom(type: .plane, size: [5, 5, 0.1])
    }
    let model = try scene.compile()
    #expect(abs(model.timestep - 0.002) < 1e-9)
}

@Test func ifInsideSceneBuilderAddsOrOmitsAnElement() throws {
    func makeScene(includeExtra: Bool) -> Scene {
        Scene {
            Geom(type: .plane, size: [5, 5, 0.1])
            if includeExtra {
                Geom(name: "extra", type: .box, size: [0.1, 0.1, 0.1])
            }
        }
    }

    let withExtra = try makeScene(includeExtra: true).compile()
    #expect(withExtra.ngeom == 2)

    let withoutExtra = try makeScene(includeExtra: false).compile()
    #expect(withoutExtra.ngeom == 1)
}

@Test func forLoopInsideSceneBuilderAddsMultipleBodies() throws {
    let scene = Scene {
        Geom(type: .plane, size: [5, 5, 0.1])
        for i in 0..<3 {
            Body(name: "box\(i)", pos: [Double(i), 0, 1]) {
                Geom(type: .box, size: [0.1, 0.1, 0.1])
            }
        }
    }
    let model = try scene.compile()
    #expect(model.nbody == 4)   // world + 3 boxes
    #expect(model.id(of: objBody, name: "box0") != nil)
    #expect(model.id(of: objBody, name: "box2") != nil)
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `swift test --filter SceneDSLTests`
Expected: FAIL to build — "cannot find 'Option' in scope" (the `if`/`for` tests should build and pass already, since `buildOptional`/`buildArray` were written in Task 1 — but leave them in this same step/commit, since this task is what actually exercises them for the first time).

- [ ] **Step 3: Implement `Option`**

Append to `Sources/MuJoCo/SceneElements.swift`:

```swift
/// Physics options. Currently just `timestep`; ignores `parent` and mutates
/// the spec directly, mirroring `MjSpec.swift`'s existing direct-field-write
/// pattern (e.g. `g!.pointee.type = ...`).
public struct Option: MjSceneElement {
    public let timestep: Double

    public init(timestep: Double) {
        self.timestep = timestep
    }

    public func apply(spec: MjSpec, parent: MjSpecBody) {
        spec.ptr.pointee.option.timestep = timestep
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `swift test --filter SceneDSLTests`
Expected: PASS (all twelve tests so far).

- [ ] **Step 5: Commit**

```bash
git add Sources/MuJoCo/SceneElements.swift Tests/MuJoCoTests/SceneDSLTests.swift
git commit -m "feat: add Option scene DSL element; exercise if/for builder control flow"
```

---

### Task 6: Parity test against the original XML, and migrate `mujoco-live-demo`

**Files:**
- Modify: `Tests/MuJoCoTests/SceneDSLTests.swift` — add the parity test.
- Modify: `Sources/mujoco-live-demo/main.swift` — replace the `xml` string literal with the DSL.

**Interfaces:**
- Consumes (from Tasks 1–5): `Scene`, `Option`, `Geom`, `Body`, `FreeJoint`. `MjModel.load(xml:)` (`Sources/MuJoCo/MjModel.swift:63`), `MjModel.geomType(_:)`/`geomSize(_:)`/`geomRgba(_:)` (`Sources/MuJoCo/MjModel.swift`), `MjModel.joints` (`Sources/MuJoCo/MjModel.swift:122`).
- Produces: nothing new — final integration task.

- [ ] **Step 1: Write the failing test**

Append to `Tests/MuJoCoTests/SceneDSLTests.swift`:

```swift
private let fallingCubeXML = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.2 0.23 0.28 1"/>
    <body name="cube" pos="0 0 2">
      <freejoint/>
      <geom name="box" type="box" size="0.15 0.15 0.15" rgba="0.2 0.6 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"""

@Test func dslSceneMatchesHandWrittenFallingCubeXML() throws {
    let dslScene = Scene {
        Option(timestep: 0.002)
        Geom(name: "floor", type: .plane, size: [5, 5, 0.1], rgba: [0.2, 0.23, 0.28, 1])
        Body(name: "cube", pos: [0, 0, 2]) {
            FreeJoint()
            Geom(name: "box", type: .box, size: [0.15, 0.15, 0.15], rgba: [0.2, 0.6, 0.9, 1])
        }
    }
    let fromDSL = try dslScene.compile()
    let fromXML = try MjModel.load(xml: fallingCubeXML)

    #expect(fromDSL.ngeom == fromXML.ngeom)
    #expect(fromDSL.nbody == fromXML.nbody)
    #expect(fromDSL.njnt == fromXML.njnt)
    #expect(abs(fromDSL.timestep - fromXML.timestep) < 1e-9)

    for i in 0..<fromDSL.ngeom {
        #expect(fromDSL.geomType(i) == fromXML.geomType(i))
        for (a, b) in zip(fromDSL.geomSize(i), fromXML.geomSize(i)) {
            #expect(abs(a - b) < 1e-9)
        }
        for (a, b) in zip(fromDSL.geomRgba(i), fromXML.geomRgba(i)) {
            #expect(abs(a - b) < 1e-5)
        }
    }
    #expect(fromDSL.joints.map(\.type) == fromXML.joints.map(\.type))

    let cubeDSL = try #require(fromDSL.id(of: objBody, name: "cube"))
    let cubeXML = try #require(fromXML.id(of: objBody, name: "cube"))
    let dDSL = MjData(fromDSL); mjForward(fromDSL, dDSL)
    let dXML = MjData(fromXML); mjForward(fromXML, dXML)
    let pDSL = dDSL.bodyPos(cubeDSL)
    let pXML = dXML.bodyPos(cubeXML)
    #expect(abs(pDSL.x - pXML.x) < 1e-9 && abs(pDSL.y - pXML.y) < 1e-9 && abs(pDSL.z - pXML.z) < 1e-9)
}
```

- [ ] **Step 2: Run the test to verify it passes**

Run: `swift test --filter SceneDSLTests`
Expected: PASS. (No new production code is needed for this test — every element it uses already exists from Tasks 1–5. This step should pass immediately; if it doesn't, that's a real bug in an earlier task, not something to patch around here.)

- [ ] **Step 3: Migrate `mujoco-live-demo`**

In `Sources/mujoco-live-demo/main.swift`, replace:

```swift
let xml = """
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.2 0.23 0.28 1"/>
    <body name="cube" pos="0 0 2">
      <freejoint/>
      <geom name="box" type="box" size="0.15 0.15 0.15" rgba="0.2 0.6 0.9 1"/>
    </body>
  </worldbody>
</mujoco>
"""
```

with:

```swift
let scene = Scene {
    Option(timestep: 0.002)
    Geom(name: "floor", type: .plane, size: [5, 5, 0.1], rgba: [0.2, 0.23, 0.28, 1])
    Body(name: "cube", pos: [0, 0, 2]) {
        FreeJoint()
        Geom(name: "box", type: .box, size: [0.15, 0.15, 0.15], rgba: [0.2, 0.6, 0.9, 1])
    }
}
```

Then update the one place that constructs the model from `xml` — find:

```swift
        let model = try MjModel.load(xml: xml)
```

and replace with:

```swift
        let model = try scene.compile()
```

- [ ] **Step 4: Build and run the demo to confirm it still works**

Run: `swift build --target mujoco-live-demo`
Expected: builds cleanly.

Run: `swift run mujoco-live-demo` (in one terminal, then Ctrl-C after confirming the startup log line appears)
Expected: prints `mujoco-live-demo: http://127.0.0.1:8788  (slot: ..., root: ...)` exactly as before — the DSL-built scene behaves identically to the XML string it replaced.

- [ ] **Step 5: Run the full test suite**

Run: `swift test`
Expected: PASS — no regressions anywhere else in the package.

- [ ] **Step 6: Commit**

```bash
git add Tests/MuJoCoTests/SceneDSLTests.swift Sources/mujoco-live-demo/main.swift
git commit -m "test: add DSL/XML parity test; migrate mujoco-live-demo off the raw XML literal"
```

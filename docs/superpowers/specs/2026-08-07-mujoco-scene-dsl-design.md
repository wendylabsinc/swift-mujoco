# MuJoCo scene DSL — design

**Date:** 2026-08-07
**Status:** Approved (brainstorming) — pending implementation plan
**Phase:** 1 of (at least) 2 — this phase is scene geometry only; entity/controller
spawning (the "worldbuilding" idea below) is future work, not designed here.

## Goal

A SwiftUI-like result-builder DSL for specifying a MuJoCo scene in Swift instead
of a raw MJCF XML string, e.g. replacing this (from
`Sources/mujoco-live-demo/main.swift`):

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
let model = try MjModel.load(xml: xml)
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
let model = try scene.compile()
```

## Non-goals (this phase)

- **Entity/controller ("brain") spawning.** The original ask expanded mid-design
  to "spawn N copies of a robot sharing a controller type, each instance
  independent — like Infrastructure as Code for sims." That's a real,
  separate design: it needs `MjSpec.attach(_:prefix:suffix:toBody:)`'s
  existing multi-robot composition (confirmed: robots should physically
  co-exist in **one shared** `MjModel`/`MjData`, not N isolated physics
  worlds), and a controller hook typed against `RobotKit.Environment`
  (`Sources/RobotKit/Environment.swift` in the not-yet-merged
  `.worktrees/robotkit-go2-locomotion` worktree) — that protocol is already
  designed as the sim/real seam
  (`docs/superpowers/specs/2026-08-07-robotkit-go2-locomotion-design.md`), so
  the follow-up should target it directly rather than invent a parallel
  abstraction. Not designed further here; revisit once `RobotKit` lands on
  `main`.
- Not a general MJCF-coverage project. This phase covers the element kinds
  `MjSpec`'s underlying C calls already expose (body, geom, joint, freejoint,
  site, camera, light) plus the `timestep` option — not actuators, tendons,
  sensors, materials, textures, meshes, or the full MJCF attribute surface.
- Not replacing `MjSpec`, `MjModel.load(xml:)`, or hand-written MJCF strings —
  all three remain valid, supported ways to build a model. The DSL is an
  additional, more ergonomic entry point that compiles down to the same
  `MjSpec`.

## Architecture

New files inside the existing `MuJoCo` SPM target (no new target/product) —
this module already depends on `CMuJoCo` and already has the raw pointer
plumbing pattern established in `MjSpec.swift`.

A single protocol drives composition — but its signature must not put raw
`Unsafe*` pointer types in public API. `Span`/`RawSpan` don't fit as a
substitute: they're non-escaping, lifetime-bound views over a *sequence* of
contiguous elements (the safe replacement for `UnsafeBufferPointer`), whereas
`mjSpec*`/`mjsBody*` here are single opaque C struct handles that must persist
and get threaded through the whole recursive tree-walk — exactly the
"escapes its borrow scope" usage Span exists to forbid. (There is no existing
`Span` usage anywhere in this codebase to follow, either.)

Instead, the pointer is hidden behind an opaque handle type whose only
stored property is `internal` (default access), so it never appears as an
`Unsafe*` type in any public signature:

```swift
/// Opaque handle to a body inside a scene under construction. Produced by
/// `Scene`'s root and by `Body.apply`; not constructible outside `MuJoCo`.
public struct MjSpecBody {
    let ptr: UnsafeMutablePointer<mjsBody>   // internal — never appears in public API
}

public protocol MjSceneElement {
    func apply(spec: MjSpec, parent: MjSpecBody)
}
```

`MjSceneElement` must be `public` so `Body`/`Geom`/etc. (all `public struct`s
conforming to it) are nameable as `[MjSceneElement]` from the builder
closures in another target's code (e.g. `mujoco-live-demo`); `MjSpecBody`
must be public for the same reason (`apply`'s parameter type must be at
least as accessible as `apply` itself). Neither exposes anything unsafe: from
outside `MuJoCo`, `MjSpecBody` is a token you can pass around but not
construct, inspect, or dereference. One consequence: `MjSceneElement`
conformances are effectively sealed to `MuJoCo` itself (a third party holding
an opaque `MjSpecBody` has nothing to call on it) — acceptable, since nothing
in this design calls for external conformances; the element set below is a
fixed, closed list.

`Scene` obtains the initial handle via `mjs_findBody(spec.ptr, "world")`,
wrapped as `MjSpecBody(ptr:)`. `Body.apply` calls `mjs_addBody(parent.ptr, nil)`,
sets name/pos/quat on the result, wraps it as a new `MjSpecBody`, and calls
`apply` on each of its own children with that as `parent` — so nesting
`Body { Body { ... } }` recurses naturally, no name-based lookup required
anywhere in the tree (unlike `MjSpec.addGeom`, which resolves its parent by
name via `mjs_findBody`; the DSL always has the real pointer in hand from the
recursion itself). Leaf elements (`Geom`, `Joint`, `FreeJoint`, `Site`,
`Camera`, `Light`) call their matching `mjs_add*` on `parent.ptr` and ignore
`spec`. `Option` ignores `parent` and mutates `spec.ptr.pointee.option.timestep`
directly (mirrors the existing `g!.pointee.type = ...` direct-field-write
pattern already used throughout `MjSpec.swift`) — this one call site does
reach through `MjSpec.ptr`, which is `MjSpec`'s own pre-existing public
escape hatch, not a new exposure this design introduces.

A result builder combines elements the SwiftUI way:

```swift
@resultBuilder
public struct MjSceneBuilder {
    static func buildBlock(_ components: [MjSceneElement]...) -> [MjSceneElement]
    static func buildOptional(_ component: [MjSceneElement]?) -> [MjSceneElement]
    static func buildEither(first: [MjSceneElement]) -> [MjSceneElement]
    static func buildEither(second: [MjSceneElement]) -> [MjSceneElement]
    static func buildArray(_ components: [[MjSceneElement]]) -> [MjSceneElement]
    static func buildExpression(_ expression: MjSceneElement) -> [MjSceneElement]
}
```

This is simpler than SwiftUI's own `ViewBuilder`: elements are heterogeneous
protocol existentials collected into a plain `[MjSceneElement]` array — no
opaque-return-type/diffing machinery is needed since nothing here re-renders,
it just walks once at compile time.

## Components

- **`Scene(@MjSceneBuilder _: () -> [MjSceneElement])`** — the root. Builds a
  **bare** `MjSpec(floor: false, light: false)` — deliberately not
  `MjSpec`'s convenience default, which would silently add its own floor
  geom and light on top of whatever the scene itself declares — then calls
  `apply` on every top-level element against the world body
  (`mjs_findBody(spec.ptr, "world")`).
  - `func spec() -> MjSpec` — the primary output. Non-throwing: it only ever
    looks up the world body on a freshly built bare spec, which `mj_makeSpec`
    guarantees always exists. Returns the live, still-editable spec (so a
    future Phase 2 can `attach()` more robots onto it, same as any other
    `MjSpec`).
  - `func compile() throws -> MjModel` — `spec().compile()`.
  - `func xml() throws -> String` — `spec().saveXML()`, for debugging/parity
    checks against hand-written MJCF.
- **`Option(timestep: Double)`**
- **`Body(name: String? = nil, pos: [Double] = [0, 0, 0], quat: [Double]? = nil, @MjSceneBuilder children: () -> [MjSceneElement] = { [] })`**
  — corrected from an earlier draft of this doc, which claimed the default
  should be `{ }` (empty body) and that `{ [] }` would fail to type-check.
  That claim was backwards, confirmed by an isolated repro: the
  `@resultBuilder` transform applies only to a closure passed at a call
  site (where braces get the special treatment SwiftUI-style DSLs rely
  on) — it does **not** apply to a default *parameter value* expression.
  There, `{ }` is just an ordinary empty closure literal, which infers as
  `() -> Void` and fails to match the declared `() -> [MjSceneElement]`;
  `{ [] }` is an ordinary closure literal returning the `[MjSceneElement]`
  array `[]` directly, no builder methods invoked at all, and type-checks
  fine. The final whole-branch review flagged the `{ [] }` vs `{ }`
  disagreement between code and docs; the fix round's implementer
  re-verified this empirically before "fixing" the code to match the
  (wrong) docs, caught the discrepancy, and left the code as `{ [] }` —
  this doc is what was actually wrong.
- **`Geom(name: String? = nil, type: MjModel.GeomType, size: [Double], pos: [Double] = [0, 0, 0], rgba: [Double] = [0.5, 0.5, 0.5, 1])`**
  — reuses `MjModel.GeomType`, already public; no new geom-type enum.
- **`FreeJoint(name: String? = nil)`** — `mjs_addFreeJoint(parent)`.
- **`Joint(name: String? = nil, type: JointKind, axis: [Double] = [0, 0, 1], pos: [Double] = [0, 0, 0], range: ClosedRange<Double>? = nil)`**
  — new small `enum JointKind { case hinge, slide, ball }` mapping to
  `mjJNT_HINGE`/`mjJNT_SLIDE`/`mjJNT_BALL` (free joints have their own
  dedicated element above, so `mjJNT_FREE` is never reached through `Joint`).
  `range` sets `limited = 1` and `range = [range.lowerBound, range.upperBound]`
  when non-nil, leaving the joint unlimited otherwise. **Units, hinge/ball
  only: degrees**, matching MJCF's own default `<compiler angle="degree">`
  convention — `Joint(type: .hinge, range: -90...90)` means -90° to 90°,
  exactly as `<joint range="-90 90"/>` would in hand-written MJCF. This is
  not a DSL-side conversion: MuJoCo's compiler applies the degree→radian
  conversion to a hinge/ball joint's `mjsJoint.range` during `mj_compile`
  regardless of whether the field was set from XML text or written directly
  (as `Joint.apply` does) — a compiled hinge/ball joint's `joints[i].range`
  is always in radians either way. **`.slide` is unaffected** — a slide
  joint's range is a linear (length) quantity, never angle-converted, and
  passes through as authored. `axis` and `pos` are also unaffected
  (direction/position values, not angle quantities).
- **`Site(name: String? = nil, pos: [Double] = [0, 0, 0], size: Double = 0.01)`**
- **`Camera(name: String? = nil, pos: [Double] = [0, 0, 0], fovy: Double = 45)`**
- **`Light(pos: [Double]? = nil)`** — `MjSpec`'s own initializer already adds
  a default light; this element is for scenes that want an explicit or
  additional one.

All `pos`/`size`/`rgba`/`axis` parameters use plain `[Double]` (rgba mixes in
`Float` at the C boundary the same way `MjSpec.addGeom` already does),
matching the convention already established by `MjSpec.addGeom`/`addSite`/
`addCamera` — no new vector type introduced.

## Error handling

`mjs_add*` calls fail only on memory exhaustion, never on ordinary input, so
every `apply` implementation force-unwraps exactly like `MjSpec.swift`
already does elsewhere (e.g. `g!.pointee.type = ...`) — no new error type for
the element-application path. The only throwing surface is `Scene.spec()`/
`compile()`/`xml()`, which is exactly `MjSpec.compile()`/`saveXML()`'s
existing `MjError` — a scene with a genuine compiler error (duplicate name,
invalid geom params, etc.) fails at the same point and with the same error
type a hand-written XML string would.

## Testing

New `Tests/MuJoCoTests/SceneDSLTests.swift`:

- **Parity test:** build the falling-cube scene above via the DSL, `compile()`
  it, and assert the result matches `MjModel.load(xml:)` on the literal XML
  string — same geom count/types/sizes/rgba, same body position, same joint
  type. This is the acceptance test proving the DSL is a drop-in replacement
  for the string it's meant to replace.
- **Nesting:** `Body { Body { ... } }` two levels deep, asserting the inner
  body's world-frame composition is correct.
- **Control flow:** an `if` and a `for` inside a `Scene`'s builder closure,
  proving `buildOptional`/`buildArray` are wired correctly (not just the
  happy-path single block).
- **Remaining elements:** one test each exercising `Joint` (hinge with a
  `range`), `Site`, `Camera`, `Light` — asserting each is present and
  correctly parameterized on the compiled `MjModel`.

## Migration

`Sources/mujoco-live-demo/main.swift`'s `xml` literal (the exact code pasted
to kick off this design) is replaced with the equivalent `Scene { ... }` +
`.compile()`, proving the DSL works against real, running code and not just
tests.

## Affected files (summary)

- New: `Sources/MuJoCo/SceneDSL.swift` (or split across `MjSceneElement.swift`
  / `Scene.swift` if it grows unwieldy as one file — implementation-time call)
- New: `Tests/MuJoCoTests/SceneDSLTests.swift`
- Modify: `Sources/mujoco-live-demo/main.swift` (XML literal → DSL)
- No `Package.swift` changes (no new target/product)

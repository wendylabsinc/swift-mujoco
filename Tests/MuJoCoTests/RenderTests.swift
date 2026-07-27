import Testing
import Foundation
@testable import MuJoCo

/// Returns true when a render test may proceed, false when it should return early.
///
/// Skipping is correct on a GL-less dev box — the suite must still go green there.
/// But a permanently-skipped render test reads as coverage while testing nothing,
/// so CI sets `SWIFT_MUJOCO_REQUIRE_GL=1`, which turns the skip into a recorded
/// failure. A runner that silently loses its Mesa stack then fails the build
/// instead of reporting a green suite that exercised no rendering.
///
/// This records an Issue rather than throwing, because a thrown error marks a
/// Swift Testing test as *failed* — which is exactly what we do NOT want on a
/// machine that legitimately has no GL.
func glAvailableOrRecordSkip() -> Bool {
    if MjOffscreenRenderer.isAvailable { return true }
    if ProcessInfo.processInfo.environment["SWIFT_MUJOCO_REQUIRE_GL"] == "1" {
        Issue.record("SWIFT_MUJOCO_REQUIRE_GL=1 but no GL context could be created")
    }
    return false
}

/// A red box dead ahead of a fixed camera, against the default background.
private let renderScene = """
<mujoco>
  <visual>
    <global offwidth="320" offheight="240"/>
  </visual>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <body name="target" pos="2 0 0">
      <geom name="redbox" type="box" size="0.5 0.5 0.5" rgba="1 0 0 1"/>
    </body>
    <camera name="eye" pos="0 0 0" xyaxes="0 -1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>
"""

@Test func cameraIntrospection() throws {
    let m = try MjModel.load(xml: renderScene)
    #expect(m.ncam == 1)
    let eye = try #require(m.id(of: objCamera, name: "eye"))
    #expect(m.camName(eye) == "eye")
    #expect(abs(m.camFovy(eye) - 45) < 1e-9)
    #expect(m.zNear > 0)
    #expect(m.zFar > m.zNear)

    // A camera with no explicit resolution reports (0, 0) — callers pick their own.
    let res = m.camResolution(eye)
    #expect(res.width >= 0 && res.height >= 0)

    // Intrinsics derived for a 320x240 target: square pixels, centred principal point.
    let k = m.camIntrinsic(eye, width: 320, height: 240)
    #expect(abs(k.cx - 160) < 1e-9)
    #expect(abs(k.cy - 120) < 1e-9)
    #expect(abs(k.fx - k.fy) < 1e-9)
    // fy = (h/2) / tan(fovy/2); fovy=45deg -> tan(22.5deg) = 0.41421
    #expect(abs(k.fy - 120.0 / 0.41421356) < 0.01)
}

@Test func offscreenRenderProducesRedCentreAndSaneDepth() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: renderScene)
    let d = MjData(m)
    mjForward(m, d)
    let eye = try #require(m.id(of: objCamera, name: "eye"))

    let r = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let frame = try r.render(data: d, cameraId: eye)

    #expect(frame.width == 320 && frame.height == 240)
    #expect(frame.rgb.count == 320 * 240 * 3)
    #expect(frame.depth.count == 320 * 240)

    // Centre pixel is the red box.
    let centre = ((240 / 2) * 320 + (320 / 2)) * 3
    #expect(frame.rgb[centre] > 100, "expected a red centre pixel, got r=\(frame.rgb[centre])")
    #expect(frame.rgb[centre + 1] < 90)
    #expect(frame.rgb[centre + 2] < 90)

    // Camera at origin, box front face at x=1.5. Depth is metres, not [0,1].
    let centreDepth = frame.depth[(240 / 2) * 320 + (320 / 2)]
    #expect(centreDepth > 1.0 && centreDepth < 2.0,
            "expected ~1.5 m to the box face, got \(centreDepth)")
}

@Test func resizeChangesFrameDimensions() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: renderScene)
    let d = MjData(m)
    mjForward(m, d)
    let r = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    try r.resize(width: 160, height: 120)
    let frame = try r.render(data: d, cameraId: 0)
    #expect(frame.width == 160 && frame.height == 120)
    #expect(frame.rgb.count == 160 * 120 * 3)
}

@Test func renderRejectsBadCameraId() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: renderScene)
    let d = MjData(m)
    mjForward(m, d)
    let r = try MjOffscreenRenderer(model: m, width: 64, height: 64)
    #expect(throws: MjError.self) { try r.render(data: d, cameraId: 99) }
}

/// A small green box offset **above** the camera's optical axis in world Z.
/// The camera's `xyaxes` maps world +Z to "up" in the image, so a correctly
/// top-down frame (row 0 = top) must show the box in the upper half of both
/// the rgb and depth buffers. Unlike `renderScene`, this scene is NOT
/// vertically symmetric about the camera axis, so it can actually distinguish
/// "flipped" from "not flipped" — and catch a flip applied to one buffer but
/// not the other.
private let asymmetricScene = """
<mujoco>
  <visual>
    <global offwidth="320" offheight="240"/>
  </visual>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <body name="target" pos="2 0 0.6">
      <geom name="greenbox" type="box" size="0.15 0.15 0.15" rgba="0 1 0 1"/>
    </body>
    <camera name="eye" pos="0 0 0" xyaxes="0 -1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>
"""

@Test func renderRowOrderIsTopDownForBothRgbAndDepth() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: asymmetricScene)
    let d = MjData(m)
    mjForward(m, d)
    let eye = try #require(m.id(of: objCamera, name: "eye"))
    let r = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let frame = try r.render(data: d, cameraId: eye)

    // Locate the greenest pixel in the frame (the box) by row.
    var bestGreenRow = -1
    var bestGreenScore = -1
    for row in 0..<frame.height {
        for col in 0..<frame.width {
            let i = (row * frame.width + col) * 3
            let g = Int(frame.rgb[i + 1])
            let rr = Int(frame.rgb[i])
            let b = Int(frame.rgb[i + 2])
            let score = g - max(rr, b)
            if g > 100 && score > bestGreenScore {
                bestGreenScore = score
                bestGreenRow = row
            }
        }
    }
    #expect(bestGreenRow >= 0, "no green pixel found in the frame at all")
    #expect(bestGreenRow < frame.height / 2,
            "box sits above the camera axis, so row 0 = top must place it in the upper half; got row \(bestGreenRow) of \(frame.height)")

    // Locate the nearest (smallest finite) depth reading by row.
    var bestDepthRow = -1
    var bestDepth = Float.infinity
    for row in 0..<frame.height {
        for col in 0..<frame.width {
            let z = frame.depth[row * frame.width + col]
            if z < bestDepth {
                bestDepth = z
                bestDepthRow = row
            }
        }
    }
    #expect(bestDepth.isFinite && bestDepth < 5.0,
            "expected the box's near depth (a few metres), got \(bestDepth)")
    #expect(bestDepthRow < frame.height / 2,
            "box sits above the camera axis, so row 0 = top must place its near depth in the upper half; got row \(bestDepthRow) of \(frame.height)")
}

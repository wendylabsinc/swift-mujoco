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

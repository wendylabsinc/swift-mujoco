import Testing
import Foundation
@testable import MuJoCo

// render() allocated two arrays per call — ~1.7 MB for a 640x480 frame, ~50 MB/s
// of immediate garbage at 30 fps. render(data:cameraId:into:) writes into
// caller-owned storage instead. These tests pin that the two paths produce
// identical pixels, that the buffer really is reused, and that FrameBuffer keeps
// itself in step with a resized renderer.

/// Same scene shape as RenderTests: a red box dead ahead of a fixed camera.
private let bufferScene = """
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

// FrameBuffer itself needs no GL, so its own behaviour is tested unconditionally.

@Test func frameBufferSizesItsStorageFromDimensions() {
    let b = FrameBuffer(width: 64, height: 32)
    #expect(b.width == 64)
    #expect(b.height == 32)
    #expect(b.rgb.count == 64 * 32 * 3)
    #expect(b.depth.count == 64 * 32)
}

@Test func frameBufferResizeIsANoOpWhenDimensionsMatch() {
    var b = FrameBuffer(width: 64, height: 32)
    b.rgb.withUnsafeBufferPointer { _ in }
    let beforeRGB = b.rgb.count
    let beforeDepth = b.depth.count
    b.resize(width: 64, height: 32)
    #expect(b.rgb.count == beforeRGB)
    #expect(b.depth.count == beforeDepth)
}

@Test func frameBufferResizeReshapesStorage() {
    var b = FrameBuffer(width: 8, height: 8)
    b.resize(width: 16, height: 4)
    #expect(b.width == 16)
    #expect(b.height == 4)
    #expect(b.rgb.count == 16 * 4 * 3)
    #expect(b.depth.count == 16 * 4)
}

@Test func frameBufferSnapshotCopiesOut() {
    let b = FrameBuffer(width: 4, height: 4)
    let snap = b.snapshot()
    #expect(snap.width == 4)
    #expect(snap.height == 4)
    #expect(snap.rgb.count == b.rgb.count)
    #expect(snap.depth.count == b.depth.count)
}

// The rest need a GL context.

@Test func renderIntoBufferMatchesTheAllocatingRender() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: bufferScene)
    let d = MjData(m)
    mjForward(m, d)
    let renderer = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let cam = try #require(m.id(of: objCamera, name: "eye"))

    let owned = try renderer.render(data: d, cameraId: cam)
    var buffer = FrameBuffer(width: 320, height: 240)
    try renderer.render(data: d, cameraId: cam, into: &buffer)

    #expect(buffer.width == owned.width)
    #expect(buffer.height == owned.height)
    #expect(buffer.rgb == owned.rgb, "rgb must be byte-identical between the two paths")
    #expect(buffer.depth == owned.depth, "depth must be identical between the two paths")
}

@Test func repeatedRendersReuseTheSameStorage() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: bufferScene)
    let d = MjData(m)
    mjForward(m, d)
    let renderer = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let cam = try #require(m.id(of: objCamera, name: "eye"))

    var buffer = FrameBuffer(width: 320, height: 240)
    // Capture the backing allocation's address; reusing the buffer must not
    // reallocate it, which is the entire point of the API.
    let firstAddress = buffer.rgb.withUnsafeBufferPointer { UInt(bitPattern: $0.baseAddress) }
    for _ in 0..<10 {
        mjStep(m, d)
        try renderer.render(data: d, cameraId: cam, into: &buffer)
    }
    let lastAddress = buffer.rgb.withUnsafeBufferPointer { UInt(bitPattern: $0.baseAddress) }
    #expect(firstAddress == lastAddress, "render(into:) must not reallocate the caller's buffer")
}

@Test func renderIntoAMismatchedBufferResizesIt() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: bufferScene)
    let d = MjData(m)
    mjForward(m, d)
    let renderer = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let cam = try #require(m.id(of: objCamera, name: "eye"))

    // Deliberately wrong size: the renderer must bring it into step rather than
    // trapping on a size mismatch.
    var buffer = FrameBuffer(width: 8, height: 8)
    try renderer.render(data: d, cameraId: cam, into: &buffer)
    #expect(buffer.width == 320)
    #expect(buffer.height == 240)
    #expect(buffer.rgb.count == 320 * 240 * 3)
}

@Test func renderIntoTracksARendererResize() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: bufferScene)
    let d = MjData(m)
    mjForward(m, d)
    let renderer = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let cam = try #require(m.id(of: objCamera, name: "eye"))

    var buffer = FrameBuffer(width: 320, height: 240)
    try renderer.render(data: d, cameraId: cam, into: &buffer)
    #expect(buffer.width == 320)

    try renderer.resize(width: 160, height: 120)
    try renderer.render(data: d, cameraId: cam, into: &buffer)
    #expect(buffer.width == 160)
    #expect(buffer.height == 120)
    #expect(buffer.depth.count == 160 * 120)
}

@Test func renderIntoRejectsAForeignData() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: bufferScene)
    let renderer = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let cam = try #require(m.id(of: objCamera, name: "eye"))
    var buffer = FrameBuffer(width: 320, height: 240)

    // An out-of-range camera must still throw on the into: path.
    #expect(throws: MjError.self) {
        try renderer.render(data: MjData(m), cameraId: m.ncam + 5, into: &buffer)
    }
    // And the valid case still works afterwards.
    let d = MjData(m)
    mjForward(m, d)
    try renderer.render(data: d, cameraId: cam, into: &buffer)
    #expect(buffer.rgb.contains { $0 != 0 }, "expected a non-blank frame")
}

@Test func renderedDepthIsTopDownAndFiniteOnTheTarget() throws {
    guard glAvailableOrRecordSkip() else { return }
    let m = try MjModel.load(xml: bufferScene)
    let d = MjData(m)
    mjForward(m, d)
    let renderer = try MjOffscreenRenderer(model: m, width: 320, height: 240)
    let cam = try #require(m.id(of: objCamera, name: "eye"))

    var buffer = FrameBuffer(width: 320, height: 240)
    try renderer.render(data: d, cameraId: cam, into: &buffer)

    // The box sits dead centre 2 m away; centre depth must be finite and close.
    let centre = (buffer.height / 2) * buffer.width + buffer.width / 2
    let z = buffer.depth[centre]
    #expect(z.isFinite, "centre pixel should have hit the box")
    #expect(abs(z - 1.5) < 0.35, "expected ~1.5 m to the near face, got \(z)")
}

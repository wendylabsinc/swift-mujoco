import CMuJoCo
import CMuJoCoGL
#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif

extension MjModel {
    /// Name of a camera, or nil for an unnamed one.
    public func camName(_ i: Int) -> String? {
        precondition(i >= 0 && i < ncam)
        return name(of: objCamera, id: i)
    }

    /// Vertical field of view in **degrees**, as MuJoCo stores it.
    public func camFovy(_ i: Int) -> Double {
        precondition(i >= 0 && i < ncam)
        return ptr.pointee.cam_fovy[i]
    }

    /// Explicit pixel resolution declared on the camera, or (0, 0) when the
    /// MJCF sets none — in which case the caller chooses.
    public func camResolution(_ i: Int) -> (width: Int, height: Int) {
        precondition(i >= 0 && i < ncam)
        guard let r = ptr.pointee.cam_resolution else { return (0, 0) }   // int*, ncam*2
        return (Int(r[i * 2 + 0]), Int(r[i * 2 + 1]))
    }

    /// Pinhole intrinsics for rendering this camera at the given size.
    ///
    /// MuJoCo cameras are ideal pinholes with square pixels and a centred
    /// principal point, so fx == fy and (cx, cy) is the image centre. Derived
    /// from fovy rather than read from `cam_intrinsic`, which is only populated
    /// for cameras declared with a physical sensor size.
    public func camIntrinsic(_ i: Int, width: Int, height: Int)
        -> (fx: Double, fy: Double, cx: Double, cy: Double) {
        precondition(i >= 0 && i < ncam)
        precondition(width > 0 && height > 0)
        let fovyRad = camFovy(i) * .pi / 180
        let fy = Double(height) / 2 / tan(fovyRad / 2)
        return (fx: fy, fy: fy, cx: Double(width) / 2, cy: Double(height) / 2)
    }

    /// Near clipping plane in metres. MuJoCo stores it as a fraction of the
    /// model's spatial extent.
    public var zNear: Double { Double(ptr.pointee.vis.map.znear) * ptr.pointee.stat.extent }
    /// Far clipping plane in metres.
    public var zFar: Double { Double(ptr.pointee.vis.map.zfar) * ptr.pointee.stat.extent }
}

/// One rendered frame. Rows are ordered **top-down** (row 0 is the top of the
/// image), matching `sensor_msgs/Image` and every common image format — MuJoCo's
/// `mjr_readPixels` returns bottom-up, and `render` flips it for you.
public struct RenderedFrame: Sendable {
    /// Packed rgb8, `width * height * 3` bytes, top-down.
    public let rgb: [UInt8]
    /// Linear depth in **metres**, `width * height` floats, top-down.
    /// A pixel that hit nothing holds `Float.infinity`.
    public let depth: [Float]
    public let width: Int
    public let height: Int
}

/// Renders MuJoCo cameras to memory with no window.
///
/// Intentionally NOT `Sendable`: it owns a GL context that is current on one
/// thread, plus mutable `mjvScene`/`mjrContext` state. Create and use it from a
/// single isolation domain — the same one that steps the model.
public final class MjOffscreenRenderer {
    /// Whether a GL context can be created on this machine. Check before
    /// constructing if a missing GL should degrade rather than throw.
    ///
    /// Probes by creating and immediately destroying a throwaway context, so
    /// it is safe to call before any renderer exists. Cached after the first
    /// call: repeated probing would otherwise clear whichever context is
    /// current on this thread (`wmj_gl_destroy` unconditionally clears the
    /// thread's current-context state), which would silently break a live
    /// renderer's next `render()` call if this were checked again afterwards.
    public static let isAvailable: Bool = {
        guard let c = wmj_gl_create() else { return false }
        wmj_gl_destroy(c)
        return true
    }()

    private let model: MjModel
    private let gl: OpaquePointer
    private var scene = mjvScene()
    private var option = mjvOption()
    private var context = mjrContext()
    public private(set) var width: Int
    public private(set) var height: Int

    // These *scratch* buffers are reused across frames — `mjr_readPixels`
    // writes into the same backing storage every call, so this class does
    // not allocate for that step. It does NOT make rendering allocation-free
    // overall: `flipVertically` and `linearizeAndFlipDepth` each build a
    // fresh array per frame, and `RenderedFrame` holds those by value (about
    // 1.7 MB per 640x480 rgb+depth frame). Making `render` itself
    // allocation-free would need `RenderedFrame` to borrow from
    // caller-owned storage instead — a larger API change, out of scope here.
    private var rgbBuffer: [UInt8]
    private var depthBuffer: [Float]

    public init(model: MjModel, width: Int, height: Int, maxGeom: Int = 10_000) throws {
        precondition(width > 0 && height > 0)
        guard let gl = wmj_gl_create() else {
            throw MjError("offscreen rendering unavailable: " + String(cString: wmj_gl_last_error()))
        }
        guard wmj_gl_make_current(gl) == 1 else {
            let e = String(cString: wmj_gl_last_error())
            wmj_gl_destroy(gl)
            throw MjError("could not make GL context current: " + e)
        }
        self.model = model
        self.gl = gl
        self.width = width
        self.height = height
        self.rgbBuffer = [UInt8](repeating: 0, count: width * height * 3)
        self.depthBuffer = [Float](repeating: 0, count: width * height)

        mjv_defaultScene(&scene)
        mjv_makeScene(model.ptr, &scene, Int32(maxGeom))
        mjv_defaultOption(&option)
        mjr_defaultContext(&context)
        mjr_makeContext(model.ptr, &context, Int32(mjFONTSCALE_100.rawValue))

        mjr_setBuffer(Int32(mjFB_OFFSCREEN.rawValue), &context)
        guard context.currentBuffer == Int32(mjFB_OFFSCREEN.rawValue) else {
            throw MjError("this GL driver has no offscreen framebuffer; MuJoCo fell back to the window buffer")
        }
        try resize(width: width, height: height)
    }

    deinit {
        // Re-assert currency before freeing: something else on this thread
        // (another renderer, an `isAvailable` probe) may have changed or
        // cleared the current context since this one was last made current.
        // Freeing GL objects against the wrong (or no) current context is
        // undefined, so this makes the deletes target the right one. The
        // return value is ignored — deinit cannot throw and there is nothing
        // better to do on failure than attempt the frees anyway.
        _ = wmj_gl_make_current(gl)
        mjr_freeContext(&context)
        mjv_freeScene(&scene)
        wmj_gl_destroy(gl)
    }

    /// Change the render size. Reallocates the pixel buffers.
    public func resize(width newWidth: Int, height newHeight: Int) throws {
        precondition(newWidth > 0 && newHeight > 0)
        try makeCurrentOrThrow()
        mjr_resizeOffscreen(Int32(newWidth), Int32(newHeight), &context)
        width = newWidth
        height = newHeight
        rgbBuffer = [UInt8](repeating: 0, count: newWidth * newHeight * 3)
        depthBuffer = [Float](repeating: 0, count: newWidth * newHeight)
    }

    /// Render the model's camera `cameraId` at the current size.
    public func render(data: MjData, cameraId: Int) throws -> RenderedFrame {
        precondition(data.model === model,
                     "MjOffscreenRenderer.render: data does not belong to the model this renderer was created with")
        guard cameraId >= 0 && cameraId < model.ncam else {
            throw MjError("render: no camera with id \(cameraId) (model has \(model.ncam))")
        }
        try makeCurrentOrThrow()
        var camera = mjvCamera()
        mjv_defaultCamera(&camera)
        camera.type = Int32(mjCAMERA_FIXED.rawValue)
        camera.fixedcamid = Int32(cameraId)

        let viewport = mjrRect(left: 0, bottom: 0, width: Int32(width), height: Int32(height))
        mjv_updateScene(model.ptr, data.ptr, &option, nil, &camera,
                        Int32(mjCAT_ALL.rawValue), &scene)
        mjr_render(viewport, &scene, &context)
        mjr_readPixels(&rgbBuffer, &depthBuffer, viewport, &context)

        return RenderedFrame(rgb: flipVertically(rgbBuffer, bytesPerPixel: 3),
                             depth: linearizeAndFlipDepth(depthBuffer),
                             width: width, height: height)
    }

    /// Re-establishes this renderer's GL context as current on the calling
    /// thread before touching any `mjv_*`/`mjr_*` state.
    ///
    /// `wmj_gl_destroy` unconditionally clears whichever context is current
    /// on the thread, regardless of which context it was destroying. So a
    /// stray `MjOffscreenRenderer.isAvailable` probe (or a second renderer on
    /// the same thread) can silently leave this renderer's context no longer
    /// current between calls. `init` makes its context current exactly once;
    /// every subsequent entry point re-asserts it here rather than assuming
    /// it is still in effect, which makes multiple renderers on one thread
    /// safe and makes any external disturbance self-healing.
    private func makeCurrentOrThrow() throws {
        guard wmj_gl_make_current(gl) == 1 else {
            throw MjError("could not make GL context current: " + String(cString: wmj_gl_last_error()))
        }
    }

    /// `mjr_readPixels` returns rows bottom-up (OpenGL convention). Flip so row
    /// 0 is the top of the image, which is what every image consumer expects.
    private func flipVertically(_ src: [UInt8], bytesPerPixel: Int) -> [UInt8] {
        let stride = width * bytesPerPixel
        var out = [UInt8](repeating: 0, count: src.count)
        for row in 0..<height {
            let from = (height - 1 - row) * stride
            let to = row * stride
            out.replaceSubrange(to..<(to + stride), with: src[from..<(from + stride)])
        }
        return out
    }

    /// OpenGL hands back a nonlinear window-space depth in [0,1]. Convert to
    /// metres and flip to top-down in one pass.
    ///
    /// `context.readDepthMap` tells us which convention `mjr_readPixels` used:
    /// `mjDEPTH_ZERONEAR` (the common case) maps raw 0 -> znear, 1 -> zfar;
    /// `mjDEPTH_ZEROFAR` (a reversed-Z map some GL drivers pick for precision
    /// via `ARB_clip_control`) maps raw 1 -> znear, 0 -> zfar. We normalise to
    /// the ZERONEAR sense before linearising so one formula covers both.
    private func linearizeAndFlipDepth(_ src: [Float]) -> [Float] {
        let znear = Float(model.zNear)
        let zfar = Float(model.zFar)
        let reversed = context.readDepthMap == Int32(mjDEPTH_ZEROFAR.rawValue)
        var out = [Float](repeating: 0, count: src.count)
        for row in 0..<height {
            let from = (height - 1 - row) * width
            let to = row * width
            for col in 0..<width {
                let raw = src[from + col]
                let z = reversed ? (1 - raw) : raw
                // z == 1 means the depth buffer was never written: nothing there.
                out[to + col] = z >= 1
                    ? .infinity
                    : znear * zfar / (zfar - z * (zfar - znear))
            }
        }
        return out
    }
}

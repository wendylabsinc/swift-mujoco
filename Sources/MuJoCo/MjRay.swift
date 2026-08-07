import CMuJoCo
#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif

/// One raycast result: which geom was struck and how far along the ray.
public struct RayHit: Equatable, Sendable {
    public let geomId: Int
    public let distance: Double
    public init(geomId: Int, distance: Double) {
        self.geomId = geomId
        self.distance = distance
    }
}

extension MjData {
    /// Cast a single ray. Returns nil when nothing is hit.
    ///
    /// - Parameters:
    ///   - geomGroupMask: bit i enables geom group i. `nil` means all groups.
    ///     Only bits 0-5 are meaningful (`mjNGROUP == 6`); bits 6-7 of the
    ///     `UInt8` are ignored.
    ///   - includeStatic: include geoms on static (worldbody) geometry.
    ///   - bodyExclude: skip all geoms on this body id; -1 excludes nothing.
    public func ray(from origin: Vec3, direction: Vec3,
                    geomGroupMask: UInt8? = nil,
                    includeStatic: Bool = true,
                    bodyExclude: Int = -1) -> RayHit? {
        var pnt = [origin.x, origin.y, origin.z]
        var vec = [direction.x, direction.y, direction.z]
        var geomid: Int32 = -1
        // VERIFIED against ~/.local/include/mujoco/mujoco.h:684-686 — mj_ray takes
        // a trailing nullable `normal[3]`. Passing nil skips normal computation.
        let dist: Double = withGeomGroup(geomGroupMask) { groupPtr in
            mj_ray(model.ptr, ptr, &pnt, &vec, groupPtr,
                   includeStatic, Int32(bodyExclude), &geomid, nil)
        }
        guard dist >= 0, geomid >= 0 else { return nil }
        return RayHit(geomId: Int(geomid), distance: dist)
    }
}

/// Expand a group bitmask into MuJoCo's `mjtByte[mjNGROUP]` form for the
/// duration of `body`. Passing nil yields a null pointer, which MuJoCo reads
/// as "all groups".
///
/// Allocates a fresh buffer per call — fine for `MjData.ray(from:)`'s
/// single-shot use, which makes no allocation-free promise. `MjRayBatch`
/// does not use this; it has its own cached `groupBuffer` (see
/// `withGroupBuffer`) since it is documented as allocating nothing per tick.
private func withGeomGroup<R>(_ mask: UInt8?, _ body: (UnsafePointer<mjtByte>?) -> R) -> R {
    guard let mask else { return body(nil) }
    var groups = [mjtByte](repeating: 0, count: Int(mjNGROUP))
    for i in 0..<Int(mjNGROUP) where (mask >> UInt8(i)) & 1 == 1 { groups[i] = 1 }
    return groups.withUnsafeBufferPointer { body($0.baseAddress) }
}

/// A reusable batched raycaster.
///
/// `mj_multiRay` requires caller-provided output buffers. This class owns them
/// so a 2,880-ray lidar sweep at 10 Hz allocates nothing per tick — which is
/// the whole reason it exists rather than a free function.
///
/// Intentionally NOT `Sendable`: the internal buffers are mutable shared state.
public final class MjRayBatch {
    /// Maximum rays a single `cast` may contain.
    public let capacity: Int
    private var geomIds: [Int32]
    private var distances: [Double]
    private var flatDirections: [Double]
    /// Reused origin buffer for `mj_multiRay`'s `pnt` argument — avoids a
    /// fresh `[Double]` literal on every call, same reasoning as `flatDirections`.
    private var pntBuffer: [Double]
    /// Reused `mjtByte[mjNGROUP]` buffer for the group mask. Populated in
    /// place from `geomGroupMask` on each call instead of allocating a fresh
    /// array — `MjRayBatch` is documented as allocation-free per tick, and
    /// `geomGroupMask` is the expected way callers avoid self-hits (see
    /// `MjRayBatch` doc comment), so this is squarely on the hot path.
    private var groupBuffer: [mjtByte]

    public init(capacity: Int) {
        precondition(capacity > 0, "MjRayBatch capacity must be positive")
        self.capacity = capacity
        self.geomIds = [Int32](repeating: -1, count: capacity)
        self.distances = [Double](repeating: -1, count: capacity)
        self.flatDirections = [Double](repeating: 0, count: capacity * 3)
        self.pntBuffer = [Double](repeating: 0, count: 3)
        self.groupBuffer = [mjtByte](repeating: 0, count: Int(mjNGROUP))
    }

    /// Populate `groupBuffer` in place from `mask` and hand a pointer to it
    /// to `body`; `nil` mask passes a null pointer ("all groups"), matching
    /// `withGeomGroup`'s semantics but without allocating.
    private func withGroupBuffer<R>(_ mask: UInt8?, _ body: (UnsafePointer<mjtByte>?) -> R) -> R {
        guard let mask else { return body(nil) }
        for i in 0..<groupBuffer.count {
            groupBuffer[i] = (mask >> UInt8(i)) & 1 == 1 ? 1 : 0
        }
        return groupBuffer.withUnsafeBufferPointer { body($0.baseAddress) }
    }

    /// Cast `directions.count` rays from a shared origin.
    ///
    /// - Returns: one entry per direction, nil where the ray hit nothing.
    ///   Allocates the returned array; use `withHits` to avoid that.
    /// - Parameter geomGroupMask: bit i enables geom group i; only bits 0-5
    ///   are meaningful (`mjNGROUP == 6`), bits 6-7 are ignored. `nil` means
    ///   all groups.
    /// - Parameter cutoff: `mj_multiRay` treats `cutoff` as a hard maximum
    ///   range: `0` means "ignore every geom", not "unlimited". `.infinity`
    ///   is the correct "no limit" default; a lidar should pass its real
    ///   `range_max` instead.
    public func cast(model: MjModel, data: MjData, origin: Vec3, directions: [Vec3],
                     geomGroupMask: UInt8? = nil,
                     includeStatic: Bool = true,
                     bodyExclude: Int = -1,
                     cutoff: Double = .infinity) -> [RayHit?] {
        precondition(model === data.model,
                     "MjRayBatch.cast: data does not belong to model")
        return withHits(model: model, data: data, origin: origin, directions: directions,
                 geomGroupMask: geomGroupMask, includeStatic: includeStatic,
                 bodyExclude: bodyExclude, cutoff: cutoff) { ids, dists in
            (0..<directions.count).map { i in
                (dists[i] >= 0 && ids[i] >= 0) ? RayHit(geomId: Int(ids[i]), distance: dists[i]) : nil
            }
        }
    }

    /// Non-allocating batched cast. `body` receives the raw geom-id and distance
    /// buffers, valid only for its duration. A distance < 0 means "no hit".
    /// - Parameter geomGroupMask: bit i enables geom group i; only bits 0-5
    ///   are meaningful (`mjNGROUP == 6`), bits 6-7 are ignored. `nil` means
    ///   all groups.
    /// - Parameter cutoff: `mj_multiRay` treats `cutoff` as a hard maximum
    ///   range: `0` means "ignore every geom", not "unlimited". `.infinity`
    ///   is the correct "no limit" default; a lidar should pass its real
    ///   `range_max` instead.
    public func withHits<R>(model: MjModel, data: MjData, origin: Vec3, directions: [Vec3],
                            geomGroupMask: UInt8? = nil,
                            includeStatic: Bool = true,
                            bodyExclude: Int = -1,
                            cutoff: Double = .infinity,
                            _ body: (UnsafeBufferPointer<Int32>, UnsafeBufferPointer<Double>) -> R) -> R {
        precondition(model === data.model,
                     "MjRayBatch.withHits: data does not belong to model")
        precondition(directions.count <= capacity,
                     "MjRayBatch: \(directions.count) rays exceeds capacity \(capacity)")
        let n = directions.count
        for (i, v) in directions.enumerated() {
            flatDirections[i*3+0] = v.x
            flatDirections[i*3+1] = v.y
            flatDirections[i*3+2] = v.z
        }
        pntBuffer[0] = origin.x
        pntBuffer[1] = origin.y
        pntBuffer[2] = origin.z
        // VERIFIED against ~/.local/include/mujoco/mujoco.h:692-694 — the argument
        // order is (…, geomid, dist, normal, nray, cutoff). `normal` sits BETWEEN
        // dist and nray and is nullable; omitting it silently shifts nray/cutoff
        // into the wrong slots and corrupts every result.
        withGroupBuffer(geomGroupMask) { groupPtr in
            mj_multiRay(model.ptr, data.ptr, &pntBuffer, &flatDirections, groupPtr,
                        includeStatic, Int32(bodyExclude),
                        &geomIds, &distances, nil, Int32(n), cutoff)
        }
        return geomIds.withUnsafeBufferPointer { ids in
            distances.withUnsafeBufferPointer { dists in
                body(UnsafeBufferPointer(start: ids.baseAddress, count: n),
                     UnsafeBufferPointer(start: dists.baseAddress, count: n))
            }
        }
    }
}

/// A spherical ray direction set for a spinning lidar.
///
/// Directions are generated **elevation-major, azimuth-minor**: all rays of the
/// lowest elevation ring first, sweeping azimuth, then the next ring up. The
/// pattern is constant for the life of a run, so it is computed once in `init`.
///
/// Both axes are **centred on 0 and endpoint-inclusive** for a partial span: a
/// 90°-azimuth pattern sweeps -45°…+45° about +x, with rays at both ends. A span
/// of a full turn (2π, within `fullTurnTolerance`) is treated as wrapping, so
/// azimuth becomes endpoint-*exclusive* — otherwise the first and last ray of
/// every ring would be duplicates. Elevation is always inclusive; a single
/// elevation ring sits at 0.
public struct LidarPattern: Sendable {
    /// How close `azimuthSpanRadians` must be to 2π to count as a wrapping full
    /// turn. Generous enough to absorb a caller passing 359.9° or a degrees→radians
    /// rounding error, tight enough not to swallow a real 350° sector.
    public static let fullTurnTolerance: Double = 1e-3

    public let azimuthCount: Int
    public let elevationCount: Int
    public let azimuthSpanRadians: Double
    public let elevationSpanRadians: Double
    /// Unit direction vectors in the sensor frame, +x forward, +z up.
    public let directions: [Vec3]

    public init(azimuthCount: Int, elevationCount: Int,
                azimuthSpanRadians: Double, elevationSpanRadians: Double) {
        precondition(azimuthCount > 0 && elevationCount > 0,
                     "LidarPattern needs at least one azimuth and one elevation step")
        self.azimuthCount = azimuthCount
        self.elevationCount = elevationCount
        self.azimuthSpanRadians = azimuthSpanRadians
        self.elevationSpanRadians = elevationSpanRadians

        // A full turn wraps, so the last ray would coincide with the first:
        // divide by count (exclusive) and start at 0. A partial sector does not
        // wrap, so it is centred on +x and includes both endpoints — matching
        // how elevation has always behaved. Getting this wrong made a 90° sector
        // sweep 0°…+88° to one side instead of -45°…+45°.
        let wraps = abs(azimuthSpanRadians.magnitude - 2 * .pi) <= Self.fullTurnTolerance
        let azStep: Double
        let azStart: Double
        if wraps {
            azStep = azimuthSpanRadians / Double(azimuthCount)
            azStart = 0
        } else if azimuthCount == 1 {
            azStep = 0
            azStart = 0
        } else {
            azStep = azimuthSpanRadians / Double(azimuthCount - 1)
            azStart = -azimuthSpanRadians / 2
        }

        var out = [Vec3]()
        out.reserveCapacity(azimuthCount * elevationCount)
        for e in 0..<elevationCount {
            // Centred on 0: a single ring sits at elevation 0.
            let elev = elevationCount == 1
                ? 0
                : -elevationSpanRadians/2 + elevationSpanRadians * Double(e) / Double(elevationCount - 1)
            let ce = cos(elev), se = sin(elev)
            for a in 0..<azimuthCount {
                let az = azStart + azStep * Double(a)
                out.append(Vec3(ce * cos(az), ce * sin(az), se))
            }
        }
        self.directions = out
    }

    /// Total rays per sweep.
    public var rayCount: Int { azimuthCount * elevationCount }
}

import Testing
@testable import MuJoCo

/// A 1x1x1 box centred at (0,0,0) plus a distant wall, for ray distance checks.
private let rayScene = """
<mujoco>
  <worldbody>
    <body name="target" pos="0 0 0">
      <geom name="cube" type="box" size="0.5 0.5 0.5"/>
    </body>
  </worldbody>
</mujoco>
"""

@Test func singleRayHitsAndMisses() throws {
    let m = try MjModel.load(xml: rayScene)
    let d = MjData(m)
    mjForward(m, d)

    // From x=-3 pointing +x: first surface of the cube is at x=-0.5, so 2.5 away.
    let hit = try #require(d.ray(from: Vec3(-3, 0, 0), direction: Vec3(1, 0, 0)))
    #expect(abs(hit.distance - 2.5) < 1e-6)
    #expect(hit.geomId == m.id(of: objGeom, name: "cube"))

    // Pointing away from the cube hits nothing.
    #expect(d.ray(from: Vec3(-3, 0, 0), direction: Vec3(-1, 0, 0)) == nil)

    // Offset far off-axis misses.
    #expect(d.ray(from: Vec3(-3, 10, 0), direction: Vec3(1, 0, 0)) == nil)
}

@Test func batchCastMatchesSingleRays() throws {
    let m = try MjModel.load(xml: rayScene)
    let d = MjData(m)
    mjForward(m, d)

    let dirs = [Vec3(1, 0, 0), Vec3(-1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)]
    let batch = MjRayBatch(capacity: dirs.count)
    let origin = Vec3(-3, 0, 0)
    let got = batch.cast(model: m, data: d, origin: origin, directions: dirs)

    #expect(got.count == 4)
    for (i, dir) in dirs.enumerated() {
        let single = d.ray(from: origin, direction: dir)
        #expect(got[i]?.geomId == single?.geomId, "ray \(i) geom")
        if let a = got[i]?.distance, let b = single?.distance {
            #expect(abs(a - b) < 1e-9, "ray \(i) distance")
        } else {
            #expect(got[i] == nil && single == nil, "ray \(i) both should miss")
        }
    }
}

@Test func batchIsReusableAcrossTicks() throws {
    let m = try MjModel.load(xml: rayScene)
    let d = MjData(m)
    mjForward(m, d)
    let batch = MjRayBatch(capacity: 8)
    let dirs = [Vec3(1, 0, 0), Vec3(0, 1, 0)]
    let first = batch.cast(model: m, data: d, origin: Vec3(-3, 0, 0), directions: dirs)
    let second = batch.cast(model: m, data: d, origin: Vec3(-3, 0, 0), directions: dirs)
    #expect(first.map(\.?.distance) == second.map(\.?.distance))
}

@Test func batchHonoursItsCapacityExactly() throws {
    let m = try MjModel.load(xml: rayScene)
    let d = MjData(m)
    mjForward(m, d)
    let batch = MjRayBatch(capacity: 2)
    #expect(batch.capacity == 2)
    // A cast exactly at capacity must work and return one entry per direction.
    // Exceeding capacity trips a `precondition`, which traps the process and so
    // cannot be asserted from inside the test runner — the guard is documented
    // in MjRayBatch.withHits rather than covered here. Do not "fix" this by
    // converting the precondition to a thrown error just to make it testable:
    // it matches the trapping contract of every other indexed accessor in this
    // library (MjData.swift:41-52).
    let got = batch.cast(model: m, data: d, origin: Vec3(0, 0, 0),
                         directions: [Vec3(1, 0, 0), Vec3(0, 1, 0)])
    #expect(got.count == 2)
}

@Test func lidarPatternShapeAndOrdering() {
    let p = LidarPattern(azimuthCount: 4, elevationCount: 3,
                         azimuthSpanRadians: 2 * .pi,
                         elevationSpanRadians: .pi / 2)
    #expect(p.directions.count == 12)
    #expect(p.azimuthCount == 4)
    #expect(p.elevationCount == 3)

    // Every direction is a unit vector.
    for v in p.directions { #expect(abs(v.norm - 1) < 1e-12) }

    // Elevation-major, azimuth-minor: the first azimuthCount entries share one
    // elevation ring, so their z components are all equal.
    let ring = p.directions[0..<4].map(\.z)
    #expect(ring.allSatisfy { abs($0 - ring[0]) < 1e-12 })
    // The next ring has a different elevation.
    #expect(abs(p.directions[4].z - p.directions[0].z) > 1e-9)

    // Cached: the same array instance-equal contents on repeat access.
    #expect(p.directions == p.directions)
}

@Test func lidarPatternSweepsFullCircle() {
    let p = LidarPattern(azimuthCount: 4, elevationCount: 1,
                         azimuthSpanRadians: 2 * .pi, elevationSpanRadians: 0)
    // 4 rays over 2*pi with a single ring at elevation 0 -> +x, +y, -x, -y.
    #expect(abs(p.directions[0].x - 1) < 1e-9)
    #expect(abs(p.directions[1].y - 1) < 1e-9)
    #expect(abs(p.directions[2].x + 1) < 1e-9)
    #expect(abs(p.directions[3].y + 1) < 1e-9)
    #expect(p.directions.allSatisfy { abs($0.z) < 1e-12 })
}

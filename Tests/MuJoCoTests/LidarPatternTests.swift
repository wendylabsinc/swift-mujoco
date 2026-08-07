import Testing
@testable import MuJoCo

#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif

// LidarPattern's azimuth axis used to be endpoint-exclusive and start at 0, while
// elevation was endpoint-inclusive and centred on 0. For a full 360° sweep
// exclusive is right (the last ray would duplicate the first), but for a partial
// FOV it meant a "90° forward" pattern actually swept 0°…+88° to one side.

/// Azimuth of a direction in the XY plane, radians.
private func azimuth(_ v: Vec3) -> Double { atan2(v.y, v.x) }
/// Elevation above the XY plane, radians.
private func elevation(_ v: Vec3) -> Double { asin(v.z / v.norm) }

@Test func partialAzimuthSpanIsCentredAndInclusive() {
    let span = Double.pi / 2      // 90° forward-facing sector
    let p = LidarPattern(azimuthCount: 5, elevationCount: 1,
                         azimuthSpanRadians: span, elevationSpanRadians: 0)
    #expect(p.directions.count == 5)
    let azimuths = p.directions.map(azimuth)
    // Endpoints are exactly ±span/2, i.e. the sector is centred on +x.
    #expect(abs(azimuths.first! - (-span / 2)) < 1e-12)
    #expect(abs(azimuths.last! - (span / 2)) < 1e-12)
    // Uniform spacing of span/(n-1).
    let step = span / 4
    for (i, az) in azimuths.enumerated() {
        #expect(abs(az - (-span / 2 + step * Double(i))) < 1e-12)
    }
    // The middle ray points straight ahead.
    #expect(abs(azimuths[2]) < 1e-12)
}

@Test func partialAzimuthSpanCoversTheFullRequestedWidth() {
    // The regression itself: the swept width must equal the requested span, not
    // (n-1)/n of it.
    let span = Double.pi / 2
    for count in [2, 3, 8, 64] {
        let p = LidarPattern(azimuthCount: count, elevationCount: 1,
                             azimuthSpanRadians: span, elevationSpanRadians: 0)
        let azimuths = p.directions.map(azimuth)
        let width = azimuths.max()! - azimuths.min()!
        #expect(abs(width - span) < 1e-12, "count=\(count) swept \(width) of \(span)")
    }
}

@Test func fullTurnStaysExclusiveSoNoRayIsDuplicated() {
    let p = LidarPattern(azimuthCount: 4, elevationCount: 1,
                         azimuthSpanRadians: 2 * .pi, elevationSpanRadians: 0)
    #expect(p.directions.count == 4)
    // 0°, 90°, 180°, 270° — the 360° ray is omitted because it is the 0° ray.
    let expected = [0.0, .pi / 2, .pi, -.pi / 2]   // atan2 wraps 270° to -90°
    for (d, want) in zip(p.directions.map(azimuth), expected) {
        #expect(abs(d - want) < 1e-12)
    }
    // No two directions coincide.
    for i in 0..<p.directions.count {
        for j in (i + 1)..<p.directions.count {
            #expect((p.directions[i] - p.directions[j]).norm > 1e-9)
        }
    }
}

@Test func nearlyFullTurnIsTreatedAsWrapping() {
    // A caller converting 359.99° from degrees should not get a duplicated ray.
    let span = 2 * Double.pi - LidarPattern.fullTurnTolerance / 2
    let p = LidarPattern(azimuthCount: 8, elevationCount: 1,
                         azimuthSpanRadians: span, elevationSpanRadians: 0)
    let first = p.directions.first!
    let last = p.directions.last!
    #expect((first - last).norm > 1e-3, "wrapping span must not duplicate the first ray")
}

@Test func realSectorJustBelowToleranceIsStillTreatedAsPartial() {
    // 350° is a real sector, not a full turn: it must stay centred/inclusive.
    let span = 350.0 * .pi / 180
    let p = LidarPattern(azimuthCount: 3, elevationCount: 1,
                         azimuthSpanRadians: span, elevationSpanRadians: 0)
    let azimuths = p.directions.map(azimuth)
    #expect(abs(azimuths[1]) < 1e-12, "middle ray points forward")
}

@Test func singleAzimuthRayPointsForward() {
    let p = LidarPattern(azimuthCount: 1, elevationCount: 1,
                         azimuthSpanRadians: .pi / 3, elevationSpanRadians: 0)
    #expect(p.directions.count == 1)
    #expect(abs(azimuth(p.directions[0])) < 1e-12)
    #expect(abs(p.directions[0].x - 1) < 1e-12)
}

@Test func elevationRemainsCentredAndInclusive() {
    let elevSpan = Double.pi / 3
    let p = LidarPattern(azimuthCount: 1, elevationCount: 3,
                         azimuthSpanRadians: 0, elevationSpanRadians: elevSpan)
    let elevations = p.directions.map(elevation)
    #expect(abs(elevations[0] - (-elevSpan / 2)) < 1e-12)
    #expect(abs(elevations[1]) < 1e-12)
    #expect(abs(elevations[2] - (elevSpan / 2)) < 1e-12)
}

@Test func orderingIsElevationMajorAzimuthMinor() {
    let p = LidarPattern(azimuthCount: 3, elevationCount: 2,
                         azimuthSpanRadians: .pi / 2, elevationSpanRadians: .pi / 6)
    #expect(p.rayCount == 6)
    #expect(p.directions.count == 6)
    // First three share the lowest elevation, last three the highest.
    let e = p.directions.map(elevation)
    #expect(abs(e[0] - e[1]) < 1e-12 && abs(e[1] - e[2]) < 1e-12)
    #expect(abs(e[3] - e[4]) < 1e-12 && abs(e[4] - e[5]) < 1e-12)
    #expect(e[0] < e[3])
}

@Test func allDirectionsAreUnitLength() {
    let p = LidarPattern(azimuthCount: 16, elevationCount: 4,
                         azimuthSpanRadians: 2 * .pi, elevationSpanRadians: .pi / 4)
    for d in p.directions {
        #expect(abs(d.norm - 1) < 1e-12)
    }
}

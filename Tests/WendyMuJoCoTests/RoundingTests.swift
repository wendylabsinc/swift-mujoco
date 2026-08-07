import Testing
import Foundation
@testable import WendyMuJoCo

// mjRound called pow(10, places) once per rounded scalar — roughly 7*ngeom times
// per state frame, for one of about six distinct values. It now uses a table. The
// results must be bit-identical to the pow implementation, and non-finite /
// overflowing inputs must not silently become NaN in the published JSON.

/// The pre-fix implementation, kept here as the reference oracle.
private func mjRoundViaPow(_ x: Double, _ places: Int) -> Double {
    let p = pow(10.0, Double(places))
    return (x * p).rounded() / p
}

@Test func roundingMatchesThePowImplementationBitForBit() {
    let values: [Double] = [
        0, 1, -1, 0.5, -0.5, 1.005, 1.2345678, -1.2345678,
        123456.789, -123456.789, 1e-7, -1e-7, 0.1, 0.2, 0.3,
        1.0 / 3, 2.0 / 3, .pi, -.pi, 9.80665, 1e12, -1e12,
    ]
    for places in 0...9 {
        for v in values {
            #expect(mjRound(v, places) == mjRoundViaPow(v, places),
                    "mjRound(\(v), \(places))")
        }
    }
}

@Test func roundingMatchesForThePlacesActuallyUsed() {
    // The call sites use 2, 4, 5 and 6 decimals.
    for places in [2, 4, 5, 6] {
        for i in -5000...5000 {
            let v = Double(i) / 997.0
            #expect(mjRound(v, places) == mjRoundViaPow(v, places), "mjRound(\(v), \(places))")
        }
    }
}

@Test func roundingKnownValues() {
    #expect(mjRound(1.234567, 2) == 1.23)
    #expect(mjRound(-0.0000004, 5) == 0.0)
    #expect(mjRound(1.5, 0) == 2.0)
    #expect(mjRound(2.5, 0) == 3.0)          // .rounded() is away-from-zero at .5
    #expect(mjRound(-1.5, 0) == -2.0)
    #expect(mjRound(0.123456789, 6) == 0.123457)
}

@Test func roundingFallsBackToPowOutsideTheTable() {
    // Beyond the table the pow path still runs, so the answers stay consistent.
    for places in [10, 11, 15] {
        #expect(mjRound(1.23456789012345, places) == mjRoundViaPow(1.23456789012345, places))
    }
    // Negative places round to tens/hundreds; still delegated to pow.
    #expect(mjRound(1234.0, -2) == mjRoundViaPow(1234.0, -2))
}

@Test func nonFiniteInputPassesThroughInsteadOfBecomingNaN() {
    // The old code multiplied first, so infinity * p = infinity, .rounded() =
    // infinity, / p = infinity — and JSONEncoder then throws on it. NaN in, NaN
    // out. Either way the caller should see what it passed in, not a silently
    // different number.
    #expect(mjRound(.infinity, 5).isInfinite)
    #expect(mjRound(-.infinity, 5).isInfinite)
    #expect(mjRound(.nan, 5).isNaN)
}

@Test func hugeFiniteInputDoesNotOverflowToInfinity() {
    // 1e300 * 1e6 overflows. Returning the input unchanged is more honest than
    // publishing infinity for a finite coordinate.
    let huge = 1e300
    #expect(mjRound(huge, 6) == huge)
    #expect(mjRound(-huge, 6) == -huge)
    #expect(mjRound(.greatestFiniteMagnitude, 2).isFinite)
}

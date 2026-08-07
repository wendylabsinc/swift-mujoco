import Foundation

/// Powers of ten for the decimal places `mjRound` is actually called with.
///
/// `pow(10, places)` was being called once per rounded scalar — roughly
/// `7 * ngeom` times per state frame — for one of six possible values. The table
/// covers 0...9 and the `pow` path stays as the fallback for anything else.
private let mjPowersOfTen: [Double] = (0...9).map { p in
    var v = 1.0
    for _ in 0..<p { v *= 10 }
    return v
}

/// Round to `places` decimals, matching Python's round() closely enough for the
/// Sim-tab JSON (used only to shrink files; the renderer parses full precision too).
func mjRound(_ x: Double, _ places: Int) -> Double {
    // Non-finite input would come back as NaN from the arithmetic below; pass it
    // through so JSON encoding fails loudly at the encoder rather than silently
    // emitting a garbage number.
    guard x.isFinite else { return x }
    let p = places >= 0 && places < mjPowersOfTen.count
        ? mjPowersOfTen[places]
        : pow(10.0, Double(places))
    let scaled = x * p
    // Overflow to infinity would turn a huge-but-finite coordinate into NaN.
    guard scaled.isFinite else { return x }
    return scaled.rounded() / p
}

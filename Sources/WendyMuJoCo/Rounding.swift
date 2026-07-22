import Foundation

/// Round to `places` decimals, matching Python's round() closely enough for the
/// Sim-tab JSON (used only to shrink files; the renderer parses full precision too).
func mjRound(_ x: Double, _ places: Int) -> Double {
    let p = pow(10.0, Double(places))
    return (x * p).rounded() / p
}

import MuJoCo
import CMuJoCo

public enum HUDValue: Encodable {
    case number(Double)
    case text(String)
    public func encode(to encoder: Encoder) throws {
        var c = encoder.singleValueContainer()
        switch self {
        case .number(let n): try c.encode(mjRound(n, 2))
        case .text(let s): try c.encode(s)
        }
    }
}

public struct StateFrame: Encodable {
    public let t: Double
    public let frame: Int
    public let engine: String
    public let pose: [[Double]]
    public let contacts: [[Double]]
    public let hud: [String: HUDValue]
    public let level: Int?   // omitted from JSON when nil (synthesized encodeIfPresent)
}

/// mjData -> per-frame poses (every geom) + bounded contacts.
public func buildState(_ model: MjModel, _ data: MjData, frame: Int,
                       hud: [String: HUDValue], level: Int?, now: Double) -> StateFrame {
    // One inner Array per geom is inherent to `pose`'s public `[[Double]]` shape
    // (and to the JSON the Sim tab parses), so it stays. What used to sit on top
    // of it does not: `geomQuat` no longer round-trips through `[Double]`/`Mat3`
    // (three more allocations per geom) and `mjRound` no longer calls `pow` seven
    // times per geom. Together with `Handle`'s publish throttle this is the
    // difference between ~2k allocations per step at 200 Hz and ~ngeom at 30 Hz.
    var pose: [[Double]] = []
    pose.reserveCapacity(model.ngeom)
    for i in 0..<model.ngeom {
        let p = data.geomXpos(i)
        let q = data.geomQuat(i)   // wxyz
        pose.append([mjRound(p.x, 5), mjRound(p.y, 5), mjRound(p.z, 5),
                     mjRound(q.w, 6), mjRound(q.x, 6), mjRound(q.y, 6), mjRound(q.z, 6)])
    }
    // Contact points (world) + linear force magnitude. Raw C: MuJoCo's wrapper exposes
    // only the normal component, but the Sim tab shows ‖linear force‖ (= norm of the
    // first 3 of mj_contactForce's 6-vector), so compute it here.
    var contacts: [[Double]] = []
    let n = Swift.min(Int(data.ptr.pointee.ncon), 64)
    if n > 0 {
        var f6 = [Double](repeating: 0, count: 6)
        for i in 0..<n {
            mj_contactForce(model.ptr, data.ptr, Int32(i), &f6)
            let mag = (f6[0]*f6[0] + f6[1]*f6[1] + f6[2]*f6[2]).squareRoot()
            let cp = data.ptr.pointee.contact[i].pos   // (Double,Double,Double)
            contacts.append([mjRound(cp.0, 4), mjRound(cp.1, 4), mjRound(cp.2, 4), mjRound(mag, 2)])
        }
    }
    return StateFrame(t: now, frame: frame, engine: "mujoco",
                      pose: pose, contacts: contacts, hud: hud, level: level)
}

// Sources/RobotKitGo2/StandController.swift
import RobotKit

/// Holds the default stand pose by commanding a zero action every tick.
///
/// Trivial on purpose: it makes the Phase 1 parity claim about the plumbing
/// rather than about a policy. Because `ActionDecoder` maps a zero action to
/// the default pose, this is also the safe fallback behavior for a robot whose
/// policy has not loaded.
public struct StandController: Controller {
    public init() {}

    public mutating func act(observation: [Float]) -> [Float] {
        [Float](repeating: 0, count: Go2Adapter.jointCount)
    }

    public mutating func reset() {}
}

import Foundation

/// A control-loop seam: `observe`/`act` work the same whether the far side is
/// a MuJoCo simulation or real hardware. Reward, training-episode
/// bookkeeping, and step-count cutoffs are NOT part of this protocol — they
/// are training-task-specific constructs computed from `Observation` by the
/// caller, not something real hardware has an intrinsic notion of.
public protocol Environment<Observation, Action> {
    associatedtype Observation
    associatedtype Action
    mutating func reset() -> Observation
    mutating func act(_ action: Action) -> Observation
    /// True when the environment considers itself done on its own terms
    /// (e.g. a fall, or an environment-intrinsic step cap) — distinct from a
    /// training harness's own `maxSteps` cutoff.
    var isTerminated: Bool { get }
}

/// Whether the current process is running inference or training. Set once at
/// the top of `main()` and read anywhere via the task-local, rather than
/// threading a mode flag through every function signature.
public enum RunMode: Sendable, Equatable {
    case infer
    case learn
}

public enum RunModeKey {
    @TaskLocal public static var current: RunMode = .infer
}

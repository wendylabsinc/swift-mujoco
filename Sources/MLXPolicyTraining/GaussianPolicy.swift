// Sources/MLXPolicyTraining/GaussianPolicy.swift
import MLX
import MLXNN
import MuJoCoRLEnv

/// `Linear(observationDimensions → hiddenDimensions) → tanh →
/// Linear(hiddenDimensions → actionDimensions)` for the action mean, plus a
/// learned per-action-dimension log-std. Used only for training (gradient
/// computation via `valueAndGrad`) — rollout-time action sampling uses
/// `snapshot()` and the MLX-free `policyForward` in `MuJoCoRLEnv` instead.
public final class GaussianPolicy: Module {
    @ModuleInfo public var fc1: Linear
    @ModuleInfo public var fc2: Linear
    public var logStd: MLXArray

    public let observationDimensions: Int
    public let hiddenDimensions: Int
    public let actionDimensions: Int

    public init(observationDimensions: Int, hiddenDimensions: Int, actionDimensions: Int) {
        self.observationDimensions = observationDimensions
        self.hiddenDimensions = hiddenDimensions
        self.actionDimensions = actionDimensions
        self.fc1 = Linear(observationDimensions, hiddenDimensions)
        self.fc2 = Linear(hiddenDimensions, actionDimensions)
        self.logStd = MLXArray.zeros([actionDimensions])
    }

    public func callAsFunction(_ x: MLXArray) -> MLXArray {
        fc2(tanh(fc1(x)))
    }
}

extension GaussianPolicy {
    /// Extracts a plain-value weight snapshot for rollout workers. `Linear`
    /// stores `weight` as `[outputDimensions, inputDimensions]` row-major —
    /// the same layout `policyForward` expects — and is always constructed
    /// with `bias: true` (the default) here, so `.bias!` is safe.
    public func snapshot() -> PolicyWeights {
        PolicyWeights(
            w1: fc1.weight.asArray(Float.self),
            b1: fc1.bias!.asArray(Float.self),
            w2: fc2.weight.asArray(Float.self),
            b2: fc2.bias!.asArray(Float.self),
            logStd: logStd.asArray(Float.self),
            inputDimensions: observationDimensions,
            hiddenDimensions: hiddenDimensions,
            actionDimensions: actionDimensions
        )
    }
}

// Sources/mujoco-rl-demo/GaussianPolicy.swift
import MLX
import MLXNN
import MuJoCoRLEnv

/// `Linear(observationDimensions → hiddenDimensions) → tanh →
/// Linear(hiddenDimensions → 1)` for the action mean, plus a learned scalar
/// log-std. Used only for training (gradient computation via `valueAndGrad`)
/// — rollout-time action sampling uses `snapshot()` and the MLX-free
/// `policyForward` in `MuJoCoRLEnv` instead.
final class GaussianPolicy: Module {
    @ModuleInfo var fc1: Linear
    @ModuleInfo var fc2: Linear
    var logStd: MLXArray

    let observationDimensions: Int
    let hiddenDimensions: Int

    init(observationDimensions: Int, hiddenDimensions: Int) {
        self.observationDimensions = observationDimensions
        self.hiddenDimensions = hiddenDimensions
        self.fc1 = Linear(observationDimensions, hiddenDimensions)
        self.fc2 = Linear(hiddenDimensions, 1)
        self.logStd = MLXArray(Float(0.0))
    }

    func callAsFunction(_ x: MLXArray) -> MLXArray {
        fc2(tanh(fc1(x)))
    }
}

extension GaussianPolicy {
    /// Extracts a plain-value weight snapshot for rollout workers. `Linear`
    /// stores `weight` as `[outputDimensions, inputDimensions]` row-major —
    /// the same layout `policyForward` expects — and is always constructed
    /// with `bias: true` (the default) here, so `.bias!` is safe.
    func snapshot() -> PolicyWeights {
        PolicyWeights(
            w1: fc1.weight.asArray(Float.self),
            b1: fc1.bias!.asArray(Float.self),
            w2: fc2.weight.asArray(Float.self),
            b2: fc2.bias!.asArray(Float.self),
            logStd: logStd.item(Float.self),
            inputDimensions: observationDimensions,
            hiddenDimensions: hiddenDimensions
        )
    }
}

// Sources/mujoco-rl-demo/PPOTrainer.swift
import MLX
import MLXNN
import MLXOptimizers
import MuJoCoRLEnv

final class ValueNetwork: Module {
    @ModuleInfo var fc1: Linear
    @ModuleInfo var fc2: Linear

    init(observationDimensions: Int, hiddenDimensions: Int) {
        self.fc1 = Linear(observationDimensions, hiddenDimensions)
        self.fc2 = Linear(hiddenDimensions, 1)
    }

    func callAsFunction(_ x: MLXArray) -> MLXArray {
        fc2(tanh(fc1(x)))
    }
}

/// Minimal clipped-surrogate PPO: no GAE, a plain per-state value baseline,
/// full-batch (no minibatching) across a fixed number of epochs per
/// iteration. The "old" log-probs are the ones `collectBatch` already
/// recorded at rollout time using this same weight snapshot, so no separate
/// old-policy pass is needed.
final class PPOTrainer {
    let policy: GaussianPolicy
    private let valueNetwork: ValueNetwork
    private let policyOptimizer: Adam
    private let valueOptimizer: Adam
    private let gamma: Float
    private let clipEpsilon: Float
    private let epochs: Int
    private let observationDimensions: Int

    init(
        observationDimensions: Int, hiddenDimensions: Int, policyLearningRate: Float,
        valueLearningRate: Float, gamma: Float, clipEpsilon: Float, epochs: Int
    ) {
        self.observationDimensions = observationDimensions
        self.policy = GaussianPolicy(observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions)
        self.valueNetwork = ValueNetwork(observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions)
        self.policyOptimizer = Adam(learningRate: policyLearningRate)
        self.valueOptimizer = Adam(learningRate: valueLearningRate)
        self.gamma = gamma
        self.clipEpsilon = clipEpsilon
        self.epochs = epochs
        eval(policy, valueNetwork)
    }

    @discardableResult
    func trainStep(trajectories: [Trajectory]) -> (policyLoss: Float, valueLoss: Float) {
        var flatObservations: [Float] = []
        var flatActions: [Float] = []
        var flatOldLogProbs: [Float] = []
        var flatReturns: [Float] = []
        for trajectory in trajectories {
            let returns = discountedReturns(rewards: trajectory.rewards, gamma: gamma)
            flatReturns.append(contentsOf: returns)
            for observation in trajectory.observations {
                flatObservations.append(contentsOf: observation)
            }
            flatActions.append(contentsOf: trajectory.actions)
            flatOldLogProbs.append(contentsOf: trajectory.logProbs)
        }
        let count = flatActions.count
        let observationsArray = MLXArray(flatObservations, [count, observationDimensions])
        let actionsArray = MLXArray(flatActions, [count, 1])
        let oldLogProbsArray = MLXArray(flatOldLogProbs, [count, 1])
        let returnsArray = MLXArray(flatReturns, [count, 1])

        let valueEstimates = valueNetwork(observationsArray)
        let advantagesArray = returnsArray - valueEstimates

        func policyLoss(model: GaussianPolicy, args: (MLXArray, MLXArray, MLXArray, MLXArray)) -> [MLXArray] {
            let (observations, actions, oldLogProbs, advantages) = args
            let mean = model(observations)
            let std = exp(model.logStd)
            let newLogProbs = gaussianLogProbMLX(actions: actions, mean: mean, std: std)
            let ratio = exp(newLogProbs - oldLogProbs)
            let surrogate1 = ratio * advantages
            let surrogate2 = clip(ratio, min: 1 - self.clipEpsilon, max: 1 + self.clipEpsilon) * advantages
            return [-minimum(surrogate1, surrogate2).mean()]
        }

        func valueLoss(model: ValueNetwork, args: (MLXArray, MLXArray)) -> [MLXArray] {
            let (observations, returns) = args
            return [square(model(observations) - returns).mean()]
        }

        let policyLossAndGrad = valueAndGrad(model: policy, policyLoss)
        let valueLossAndGrad = valueAndGrad(model: valueNetwork, valueLoss)

        var lastPolicyLoss: Float = 0
        var lastValueLoss: Float = 0
        for _ in 0..<epochs {
            let (policyLosses, policyGradients) = policyLossAndGrad(
                policy, (observationsArray, actionsArray, oldLogProbsArray, advantagesArray)
            )
            policyOptimizer.update(model: policy, gradients: policyGradients)
            eval(policy, policyOptimizer)
            lastPolicyLoss = policyLosses[0].item(Float.self)

            let (valueLosses, valueGradients) = valueLossAndGrad(valueNetwork, (observationsArray, returnsArray))
            valueOptimizer.update(model: valueNetwork, gradients: valueGradients)
            eval(valueNetwork, valueOptimizer)
            lastValueLoss = valueLosses[0].item(Float.self)
        }

        return (lastPolicyLoss, lastValueLoss)
    }
}

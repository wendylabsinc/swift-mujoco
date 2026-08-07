// Sources/MLXPolicyTraining/ReinforceTrainer.swift
import MLX
import MLXNN
import MLXOptimizers
import MuJoCoRLEnv

public final class ReinforceTrainer {
    public let policy: GaussianPolicy
    private let optimizer: Adam
    private let gamma: Float
    private let observationDimensions: Int
    private let actionDimensions: Int

    public init(
        observationDimensions: Int, hiddenDimensions: Int, actionDimensions: Int,
        learningRate: Float, gamma: Float
    ) {
        self.observationDimensions = observationDimensions
        self.actionDimensions = actionDimensions
        self.policy = GaussianPolicy(
            observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
            actionDimensions: actionDimensions
        )
        self.optimizer = Adam(learningRate: learningRate)
        self.gamma = gamma
        eval(policy)
    }

    /// Discounted returns with a batch-mean baseline; one Adam step over the
    /// whole batch.
    @discardableResult
    public func trainStep(trajectories: [Trajectory]) -> Float {
        var flatObservations: [Float] = []
        var flatActions: [Float] = []
        var flatReturns: [Float] = []
        for trajectory in trajectories {
            let returns = discountedReturns(rewards: trajectory.rewards, gamma: gamma)
            flatReturns.append(contentsOf: returns)
            for observation in trajectory.observations {
                flatObservations.append(contentsOf: observation)
            }
            for action in trajectory.actions {
                flatActions.append(contentsOf: action)
            }
        }
        let count = trajectories.reduce(0) { $0 + $1.actions.count }
        let baseline = flatReturns.reduce(0, +) / Float(flatReturns.count)
        let advantages = flatReturns.map { $0 - baseline }

        let observationsArray = MLXArray(flatObservations, [count, observationDimensions])
        let actionsArray = MLXArray(flatActions, [count, actionDimensions])
        let advantagesArray = MLXArray(advantages, [count, 1])

        func loss(model: GaussianPolicy, args: (MLXArray, MLXArray, MLXArray)) -> [MLXArray] {
            let (observations, actions, advantages) = args
            let mean = model(observations)
            let std = exp(model.logStd)
            let logProb = gaussianLogProbMLX(actions: actions, mean: mean, std: std)
            let lossValue = -(logProb * advantages).mean()
            return [lossValue]
        }

        let lossAndGrad = valueAndGrad(model: policy, loss)
        let (losses, gradients) = lossAndGrad(policy, (observationsArray, actionsArray, advantagesArray))
        optimizer.update(model: policy, gradients: gradients)
        eval(policy, optimizer)
        return losses[0].item(Float.self)
    }
}

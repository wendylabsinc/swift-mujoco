// Sources/mujoco-rl-demo/main.swift
import MuJoCoRLEnv
import MLXPolicyTraining

let algorithm = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "reinforce"
let observationDimensions = 4
let actionDimensions = 1
let hiddenDimensions = 32
let gamma: Float = 0.99
let episodesPerBatch = 16
let iterations = 200

func meanReturn(_ trajectories: [Trajectory]) -> Float {
    let totals = trajectories.map { $0.rewards.reduce(0, +) }
    return totals.reduce(0, +) / Float(totals.count)
}

func cartpoleReward(_: CartpoleObservation) -> Float { 1.0 }

switch algorithm {
case "reinforce":
    let trainer = ReinforceTrainer(
        observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
        actionDimensions: actionDimensions, learningRate: 3e-3, gamma: gamma
    )
    print("Training REINFORCE on cartpole balance (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
    for iteration in 0..<iterations {
        let weights = trainer.policy.snapshot()
        let trajectories = await collectBatch(
            makeEnvironment: { CartpoleEnv() }, weights: weights, episodeCount: episodesPerBatch,
            reward: cartpoleReward, maxSteps: CartpoleEnv.maxSteps, baseSeed: UInt64(iteration) &* 1_000_003
        )
        let loss = trainer.trainStep(trajectories: trajectories)
        print("iter \(iteration): mean return \(meanReturn(trajectories))  (loss \(loss))")
    }
case "ppo":
    let trainer = PPOTrainer(
        observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
        actionDimensions: actionDimensions, policyLearningRate: 3e-3, valueLearningRate: 1e-2,
        gamma: gamma, clipEpsilon: 0.2, epochs: 4
    )
    print("Training PPO on cartpole balance (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
    for iteration in 0..<iterations {
        let weights = trainer.policy.snapshot()
        let trajectories = await collectBatch(
            makeEnvironment: { CartpoleEnv() }, weights: weights, episodeCount: episodesPerBatch,
            reward: cartpoleReward, maxSteps: CartpoleEnv.maxSteps, baseSeed: UInt64(iteration) &* 1_000_003
        )
        let (policyLoss, valueLoss) = trainer.trainStep(trajectories: trajectories)
        print("iter \(iteration): mean return \(meanReturn(trajectories))  (policy loss \(policyLoss), value loss \(valueLoss))")
    }
default:
    print("Unknown algorithm '\(algorithm)'. Usage: mujoco-rl-demo [reinforce|ppo]")
}

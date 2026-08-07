// Sources/mujoco-rl-demo/main.swift
import MuJoCoRLEnv

let algorithm = CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "reinforce"
let observationDimensions = 4
let hiddenDimensions = 32
let gamma: Float = 0.99
let episodesPerBatch = 16
let iterations = 200

func meanReturn(_ trajectories: [Trajectory]) -> Float {
    let totals = trajectories.map { $0.rewards.reduce(0, +) }
    return totals.reduce(0, +) / Float(totals.count)
}

switch algorithm {
case "reinforce":
    let trainer = ReinforceTrainer(
        observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
        learningRate: 3e-3, gamma: gamma
    )
    print("Training REINFORCE on cartpole balance (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
    for iteration in 0..<iterations {
        let weights = trainer.policy.snapshot()
        let trajectories = await collectBatch(
            weights: weights, episodeCount: episodesPerBatch, baseSeed: UInt64(iteration) &* 1_000_003
        )
        let loss = trainer.trainStep(trajectories: trajectories)
        print("iter \(iteration): mean return \(meanReturn(trajectories))  (loss \(loss))")
    }
default:
    print("Unknown algorithm '\(algorithm)'. Usage: mujoco-rl-demo [reinforce|ppo]")
}

// Tests/MLXPolicyTrainingTests/PPOTrainerTests.swift
import Testing

@testable import MLXPolicyTraining
import MuJoCoRLEnv

@Test func trainStepReducesValueLossOnAFixedBatch() {
    let trainer = PPOTrainer(
        observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 1,
        policyLearningRate: 1e-2, valueLearningRate: 1e-1, gamma: 0.99, clipEpsilon: 0.2, epochs: 4
    )
    let observations = (0..<20).map { i in [Float(i) * 0.01, 0, 0.02, 0] }
    let actions = (0..<20).map { i in [Float(i) * 0.05 - 0.5] }
    let logProbs = observations.indices.map { i in
        gaussianLogProb(action: actions[i], mean: [0], std: [1])
    }
    let trajectories = [
        Trajectory(
            observations: observations, actions: actions, logProbs: logProbs,
            rewards: [Float](repeating: 1, count: 20)
        )
    ]

    let (_, firstValueLoss) = trainer.trainStep(trajectories: trajectories)
    var lastValueLoss = firstValueLoss
    for _ in 0..<19 {
        (_, lastValueLoss) = trainer.trainStep(trajectories: trajectories)
    }

    #expect(lastValueLoss < firstValueLoss)
}

@Test func trainStepHandlesMultiDimensionalActions() {
    let trainer = PPOTrainer(
        observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 3,
        policyLearningRate: 1e-2, valueLearningRate: 1e-1, gamma: 0.99, clipEpsilon: 0.2, epochs: 4
    )
    let observations = (0..<20).map { i in [Float(i) * 0.01, 0, 0.02, 0] }
    let actions = (0..<20).map { i in [Float(i) * 0.05 - 0.5, 0, 0] }
    let logProbs = observations.indices.map { i in
        gaussianLogProb(action: actions[i], mean: [0, 0, 0], std: [1, 1, 1])
    }
    let trajectories = [
        Trajectory(
            observations: observations, actions: actions, logProbs: logProbs,
            rewards: [Float](repeating: 1, count: 20)
        )
    ]
    let (_, firstValueLoss) = trainer.trainStep(trajectories: trajectories)
    var lastValueLoss = firstValueLoss
    for _ in 0..<19 {
        (_, lastValueLoss) = trainer.trainStep(trajectories: trajectories)
    }
    #expect(lastValueLoss < firstValueLoss)
}

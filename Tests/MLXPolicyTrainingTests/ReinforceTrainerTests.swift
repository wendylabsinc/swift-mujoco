// Tests/MLXPolicyTrainingTests/ReinforceTrainerTests.swift
import Testing

@testable import MLXPolicyTraining
import MuJoCoRLEnv

@Test func trainStepReducesLossOnAFixedBatch() {
    let trainer = ReinforceTrainer(
        observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 1, learningRate: 1e-2, gamma: 0.99
    )
    let trajectories = [
        Trajectory(
            observations: (0..<20).map { i in [Float(i) * 0.01, 0, 0.02, 0] },
            actions: (0..<20).map { i in [Float(i) * 0.05 - 0.5] },
            logProbs: [Float](repeating: 0, count: 20),
            rewards: [Float](repeating: 1, count: 20)
        )
    ]

    let firstLoss = trainer.trainStep(trajectories: trajectories)
    var lastLoss = firstLoss
    for _ in 0..<19 {
        lastLoss = trainer.trainStep(trajectories: trajectories)
    }

    #expect(lastLoss < firstLoss)
}

// Tests/MLXPolicyTrainingTests/GaussianPolicyTests.swift
import MLX
import Testing

@testable import MLXPolicyTraining
import MuJoCoRLEnv

@Test func snapshotMatchesLiveMLXForwardPass() {
    let policy = GaussianPolicy(observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 1)
    let weights = policy.snapshot()

    let observation: [Float] = [0.1, -0.2, 0.3, -0.4]
    let mlxOutput = policy(MLXArray(observation, [1, 4]))
    let mlxMean = mlxOutput.item(Float.self)

    let (handRolledMean, _) = policyForward(weights, observation: observation)

    #expect(abs(mlxMean - handRolledMean[0]) < 1e-4)
}

@Test func snapshotShapesMatchDeclaredDimensions() {
    let policy = GaussianPolicy(observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 1)
    let weights = policy.snapshot()
    #expect(weights.w1.count == 8 * 4)
    #expect(weights.b1.count == 8)
    #expect(weights.w2.count == 8)
    #expect(weights.b2.count == 1)
    #expect(weights.logStd.count == 1)
    #expect(weights.inputDimensions == 4)
    #expect(weights.hiddenDimensions == 8)
    #expect(weights.actionDimensions == 1)
}

@Test func snapshotShapesScaleWithMultipleActionDimensions() {
    let policy = GaussianPolicy(observationDimensions: 4, hiddenDimensions: 8, actionDimensions: 12)
    let weights = policy.snapshot()
    #expect(weights.w2.count == 12 * 8)
    #expect(weights.b2.count == 12)
    #expect(weights.logStd.count == 12)
    #expect(weights.actionDimensions == 12)
}

@Test func gaussianLogProbMLXMatchesScalarImplementation() {
    let mean: Float = 0.5
    let std: Float = 1.2
    let action: Float = -0.3
    let mlxResult = gaussianLogProbMLX(
        actions: MLXArray([action], [1, 1]),
        mean: MLXArray([mean], [1, 1]),
        std: MLXArray([std], [1, 1])
    ).item(Float.self)
    let scalarResult = gaussianLogProb(action: [action], mean: [mean], std: [std])
    #expect(abs(mlxResult - scalarResult) < 1e-4)
}

@Test func gaussianLogProbMLXSumsAcrossActionDimensions() {
    let actions = MLXArray([Float(0), Float(0)], [1, 2])
    let mean = MLXArray([Float(0), Float(0)], [1, 2])
    let std = MLXArray([Float(1), Float(1)], [1, 2])
    let mlxResult = gaussianLogProbMLX(actions: actions, mean: mean, std: std).item(Float.self)
    let scalarResult = gaussianLogProb(action: [0, 0], mean: [0, 0], std: [1, 1])
    #expect(abs(mlxResult - scalarResult) < 1e-4)
}

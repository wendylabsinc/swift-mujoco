// Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift
import Testing

@testable import MuJoCoRLEnv

@Test func forwardPassMatchesHandComputedValue() {
    // hiddenDimensions=2, inputDimensions=2, w1 = identity, b1 = 0,
    // w2 = [1, 1], b2 = 0, logStd = 0.
    // hidden = tanh([1, 1]) ≈ [0.7615941559557649, 0.7615941559557649]
    // mean = hidden[0] + hidden[1] ≈ 1.5231883119115298 ; std = exp(0) = 1
    let weights = PolicyWeights(
        w1: [1, 0, 0, 1], b1: [0, 0], w2: [1, 1], b2: [0], logStd: 0,
        inputDimensions: 2, hiddenDimensions: 2
    )
    let (mean, std) = policyForward(weights, observation: [1, 1])
    #expect(abs(mean - 1.5231883) < 1e-4)
    #expect(abs(std - 1.0) < 1e-6)
}

@Test func gaussianLogProbMatchesClosedForm() {
    // logProb(0 | mean=0, std=1) = -0.5 * log(2*pi) ≈ -0.9189385332
    let logProb = gaussianLogProb(action: 0, mean: 0, std: 1)
    #expect(abs(logProb - (-0.9189385)) < 1e-4)
}

@Test func splitMix64IsDeterministicGivenASeed() {
    var rngA = SplitMix64(seed: 42)
    var rngB = SplitMix64(seed: 42)
    #expect(rngA.next() == rngB.next())
    #expect(rngA.next() == rngB.next())
}

@Test func sampleGaussianIsReproducibleForTheSameSeed() {
    var rngA = SplitMix64(seed: 7)
    var rngB = SplitMix64(seed: 7)
    let a = sampleGaussian(mean: 0, std: 1, using: &rngA)
    let b = sampleGaussian(mean: 0, std: 1, using: &rngB)
    #expect(a == b)
}

@Test func sampleGaussianWithZeroStdReturnsTheMean() {
    var rng = SplitMix64(seed: 1)
    let action = sampleGaussian(mean: 3.5, std: 0, using: &rng)
    #expect(action == 3.5)
}

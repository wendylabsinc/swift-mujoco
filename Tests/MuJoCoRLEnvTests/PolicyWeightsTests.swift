// Tests/MuJoCoRLEnvTests/PolicyWeightsTests.swift
#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif
import Testing

@testable import MuJoCoRLEnv

@Test func forwardPassMatchesHandComputedValue() {
    // hiddenDimensions=2, inputDimensions=2, w1 = identity, b1 = 0,
    // w2 = [1, 1] (one row, actionDimensions=1), b2 = [0], logStd = [0].
    // hidden = tanh([1, 1]) ≈ [0.7615941559557649, 0.7615941559557649]
    // mean[0] = hidden[0] + hidden[1] ≈ 1.5231883119115298 ; std[0] = exp(0) = 1
    let weights = PolicyWeights(
        w1: [1, 0, 0, 1], b1: [0, 0], w2: [1, 1], b2: [0], logStd: [0],
        inputDimensions: 2, hiddenDimensions: 2, actionDimensions: 1
    )
    let (mean, std) = policyForward(weights, observation: [1, 1])
    #expect(mean.count == 1)
    #expect(abs(mean[0] - 1.5231883) < 1e-4)
    #expect(abs(std[0] - 1.0) < 1e-6)
}

@Test func forwardPassHandlesMultipleActionDimensionsIndependently() {
    // hiddenDimensions=2, inputDimensions=2, w1 = identity, b1 = 0,
    // actionDimensions=2: row0 = [1, 0] (only reads hidden[0]), row1 = [0, 1] (only reads hidden[1]).
    // hidden ≈ [0.7615941559557649, 0.7615941559557649]
    let weights = PolicyWeights(
        w1: [1, 0, 0, 1], b1: [0, 0], w2: [1, 0, 0, 1], b2: [0, 0], logStd: [0, 1],
        inputDimensions: 2, hiddenDimensions: 2, actionDimensions: 2
    )
    let (mean, std) = policyForward(weights, observation: [1, 1])
    #expect(mean.count == 2)
    #expect(abs(mean[0] - 0.7615941) < 1e-4)
    #expect(abs(mean[1] - 0.7615941) < 1e-4)
    #expect(abs(std[0] - 1.0) < 1e-6)
    #expect(abs(std[1] - exp(Float(1))) < 1e-4)
}

@Test func gaussianLogProbMatchesClosedForm() {
    // logProb(0 | mean=0, std=1) = -0.5 * log(2*pi) ≈ -0.9189385332
    let logProb = gaussianLogProb(action: [0], mean: [0], std: [1])
    #expect(abs(logProb - (-0.9189385)) < 1e-4)
}

@Test func gaussianLogProbSumsIndependentDimensions() {
    // Two independent standard-normal dimensions at 0: each contributes -0.9189385332.
    let logProb = gaussianLogProb(action: [0, 0], mean: [0, 0], std: [1, 1])
    #expect(abs(logProb - 2 * (-0.9189385)) < 1e-4)
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
    let a = sampleGaussian(mean: [0], std: [1], using: &rngA)
    let b = sampleGaussian(mean: [0], std: [1], using: &rngB)
    #expect(a == b)
}

@Test func sampleGaussianWithZeroStdReturnsTheMean() {
    var rng = SplitMix64(seed: 1)
    let action = sampleGaussian(mean: [3.5, -1.0], std: [0, 0], using: &rng)
    #expect(action == [3.5, -1.0])
}

import Testing

@testable import MuJoCoRLEnv

@Test func discountedReturnsMatchHandComputedValues() {
    // rewards = [1, 1, 1], gamma = 0.5
    // r[2] = 1 ; r[1] = 1 + 0.5*1 = 1.5 ; r[0] = 1 + 0.5*1.5 = 1.75
    let returns = discountedReturns(rewards: [1, 1, 1], gamma: 0.5)
    #expect(returns.count == 3)
    #expect(abs(returns[0] - 1.75) < 1e-6)
    #expect(abs(returns[1] - 1.5) < 1e-6)
    #expect(abs(returns[2] - 1.0) < 1e-6)
}

private let zeroWeightPolicy = PolicyWeights(
    w1: [Float](repeating: 0, count: 4 * 32), b1: [Float](repeating: 0, count: 32),
    w2: [Float](repeating: 0, count: 32), b2: [0], logStd: 0,
    inputDimensions: 4, hiddenDimensions: 32
)

@Test func collectEpisodeProducesConsistentArrayLengths() {
    let trajectory = collectEpisode(weights: zeroWeightPolicy, seed: 1)
    #expect(trajectory.observations.count > 0)
    #expect(trajectory.observations.count <= CartpoleEnv.maxSteps)
    #expect(trajectory.actions.count == trajectory.observations.count)
    #expect(trajectory.logProbs.count == trajectory.observations.count)
    #expect(trajectory.rewards.count == trajectory.observations.count)
    #expect(trajectory.observations.allSatisfy { $0.count == 4 })
}

@Test func collectBatchRunsEpisodesInParallel() async {
    let trajectories = await collectBatch(weights: zeroWeightPolicy, episodeCount: 4, baseSeed: 100)
    #expect(trajectories.count == 4)
    for trajectory in trajectories {
        #expect(trajectory.observations.count > 0)
    }
}

@Test func collectEpisodeIsDeterministicForAFixedSeed() {
    let a = collectEpisode(weights: zeroWeightPolicy, seed: 55)
    let b = collectEpisode(weights: zeroWeightPolicy, seed: 55)
    #expect(a.actions == b.actions)
    #expect(a.rewards == b.rewards)
}

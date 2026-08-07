public struct Trajectory: Sendable {
    public var observations: [[Float]]
    public var actions: [Float]
    public var logProbs: [Float]
    public var rewards: [Float]

    public init(
        observations: [[Float]] = [], actions: [Float] = [], logProbs: [Float] = [],
        rewards: [Float] = []
    ) {
        self.observations = observations
        self.actions = actions
        self.logProbs = logProbs
        self.rewards = rewards
    }
}

/// Reward-to-go with discount `gamma`, computed backward through the episode.
public func discountedReturns(rewards: [Float], gamma: Float) -> [Float] {
    var returns = [Float](repeating: 0, count: rewards.count)
    var runningReturn: Float = 0
    for t in stride(from: rewards.count - 1, through: 0, by: -1) {
        runningReturn = rewards[t] + gamma * runningReturn
        returns[t] = runningReturn
    }
    return returns
}

/// Runs one full episode against a fresh `CartpoleEnv`, sampling actions
/// from `weights` via the hand-rolled forward pass. Safe to call
/// concurrently from multiple tasks: nothing here is shared mutable state,
/// and the `CartpoleEnv` this function creates never leaves it.
public func collectEpisode(weights: PolicyWeights, seed: UInt64) -> Trajectory {
    var rng = SplitMix64(seed: seed)
    let env = CartpoleEnv()
    var trajectory = Trajectory()
    var observation = env.reset()
    while true {
        let observationArray = observation.asArray
        let (mean, std) = policyForward(weights, observation: observationArray)
        let action = sampleGaussian(mean: mean, std: std, using: &rng)
        let logProb = gaussianLogProb(action: action, mean: mean, std: std)
        let (nextObservation, reward, done) = env.step(action: action)

        trajectory.observations.append(observationArray)
        trajectory.actions.append(action)
        trajectory.logProbs.append(logProb)
        trajectory.rewards.append(reward)

        observation = nextObservation
        if done { break }
    }
    return trajectory
}

/// Fans out `episodeCount` independent episodes across a `TaskGroup`. Each
/// worker gets a distinct seed (`baseSeed + index`) so parallel workers don't
/// duplicate the same action sequence.
public func collectBatch(weights: PolicyWeights, episodeCount: Int, baseSeed: UInt64) async -> [Trajectory] {
    await withTaskGroup(of: Trajectory.self) { group in
        for index in 0..<episodeCount {
            group.addTask {
                collectEpisode(weights: weights, seed: baseSeed &+ UInt64(index))
            }
        }
        var trajectories: [Trajectory] = []
        trajectories.reserveCapacity(episodeCount)
        for await trajectory in group {
            trajectories.append(trajectory)
        }
        return trajectories
    }
}

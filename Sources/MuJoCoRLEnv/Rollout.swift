import RobotKit

/// Anything that can flatten itself into the policy network's input vector.
public protocol ObservationEncoding {
    var asArray: [Float] { get }
}

public struct Trajectory: Sendable {
    public var observations: [[Float]]
    public var actions: [[Float]]
    public var logProbs: [Float]
    public var rewards: [Float]

    public init(
        observations: [[Float]] = [], actions: [[Float]] = [], logProbs: [Float] = [],
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

/// Runs one full episode against a fresh `Environment`, sampling actions
/// from `weights` via the hand-rolled forward pass. Safe to call
/// concurrently from multiple tasks: nothing here is shared mutable state,
/// and the environment this function creates never leaves it. Termination is
/// `env.isTerminated || steps >= maxSteps` — `reward` is the only
/// caller-supplied piece, since it's genuinely training-task-specific (the
/// same environment could be trained against different reward shaping),
/// unlike termination, which the environment already knows how to answer.
public func collectEpisode<E: Environment>(
    makeEnvironment: () -> E, weights: PolicyWeights,
    reward: (E.Observation) -> Float, maxSteps: Int, seed: UInt64
) -> Trajectory where E.Action == [Float], E.Observation: ObservationEncoding {
    var rng = SplitMix64(seed: seed)
    var env = makeEnvironment()
    var trajectory = Trajectory()
    var observation = env.reset()
    var steps = 0
    while true {
        let observationArray = observation.asArray
        let (mean, std) = policyForward(weights, observation: observationArray)
        let action = sampleGaussian(mean: mean, std: std, using: &rng)
        let logProb = gaussianLogProb(action: action, mean: mean, std: std)
        let nextObservation = env.act(action)
        steps += 1

        trajectory.observations.append(observationArray)
        trajectory.actions.append(action)
        trajectory.logProbs.append(logProb)
        trajectory.rewards.append(reward(nextObservation))

        observation = nextObservation
        if env.isTerminated || steps >= maxSteps { break }
    }
    return trajectory
}

/// Fans out `episodeCount` independent episodes across a `TaskGroup`. Each
/// worker gets a distinct seed (`baseSeed + index`) so parallel workers don't
/// duplicate the same action sequence, and constructs its own `Environment`
/// via `makeEnvironment()` — never sharing one across the task boundary.
public func collectBatch<E: Environment>(
    makeEnvironment: @escaping @Sendable () -> E, weights: PolicyWeights, episodeCount: Int,
    reward: @escaping @Sendable (E.Observation) -> Float, maxSteps: Int, baseSeed: UInt64
) async -> [Trajectory] where E.Action == [Float], E.Observation: ObservationEncoding {
    await withTaskGroup(of: Trajectory.self) { group in
        for index in 0..<episodeCount {
            group.addTask {
                collectEpisode(
                    makeEnvironment: makeEnvironment, weights: weights, reward: reward,
                    maxSteps: maxSteps, seed: baseSeed &+ UInt64(index)
                )
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

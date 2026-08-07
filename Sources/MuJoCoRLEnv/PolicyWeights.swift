#if canImport(Glibc)
import Glibc
#else
import Darwin
#endif

/// A plain-value snapshot of a Gaussian policy's weights: `Linear(in→hidden)
/// → tanh → Linear(hidden→actionDimensions)` for the action mean, plus one
/// log-std per action dimension (diagonal-covariance Gaussian). `Sendable` by
/// construction (all stored properties are value types), so it can cross a
/// `TaskGroup` boundary into parallel rollout workers with no MLX object ever
/// doing the same.
public struct PolicyWeights: Sendable {
    /// Row-major [hiddenDimensions x inputDimensions].
    public let w1: [Float]
    public let b1: [Float]
    /// Row-major [actionDimensions x hiddenDimensions].
    public let w2: [Float]
    public let b2: [Float]
    public let logStd: [Float]
    public let inputDimensions: Int
    public let hiddenDimensions: Int
    public let actionDimensions: Int

    public init(
        w1: [Float], b1: [Float], w2: [Float], b2: [Float], logStd: [Float],
        inputDimensions: Int, hiddenDimensions: Int, actionDimensions: Int
    ) {
        self.w1 = w1
        self.b1 = b1
        self.w2 = w2
        self.b2 = b2
        self.logStd = logStd
        self.inputDimensions = inputDimensions
        self.hiddenDimensions = hiddenDimensions
        self.actionDimensions = actionDimensions
    }
}

/// The rollout-time policy forward pass. Deliberately not MLX: this runs
/// inside parallel `TaskGroup` workers, and MLX's thread-safety for
/// concurrent forward passes across OS threads is unverified. Gradients are
/// only ever needed at training time (see `MLXPolicyTraining`), so this
/// plain arithmetic version is sufficient here.
public func policyForward(_ weights: PolicyWeights, observation: [Float]) -> (mean: [Float], std: [Float]) {
    precondition(observation.count == weights.inputDimensions)
    var hidden = [Float](repeating: 0, count: weights.hiddenDimensions)
    for h in 0..<weights.hiddenDimensions {
        var sum = weights.b1[h]
        for i in 0..<weights.inputDimensions {
            sum += weights.w1[h * weights.inputDimensions + i] * observation[i]
        }
        hidden[h] = tanh(sum)
    }
    var mean = [Float](repeating: 0, count: weights.actionDimensions)
    for a in 0..<weights.actionDimensions {
        var sum = weights.b2[a]
        for h in 0..<weights.hiddenDimensions {
            sum += weights.w2[a * weights.hiddenDimensions + h] * hidden[h]
        }
        mean[a] = sum
    }
    let std = weights.logStd.map { exp($0) }
    return (mean, std)
}

/// Joint log-probability under a diagonal-covariance Gaussian: the sum of
/// each action dimension's independent log-probability.
public func gaussianLogProb(action: [Float], mean: [Float], std: [Float]) -> Float {
    precondition(action.count == mean.count && mean.count == std.count)
    var total: Float = 0
    for i in 0..<action.count {
        let variance = std[i] * std[i]
        total += -0.5 * log(2 * Float.pi * variance) - (action[i] - mean[i]) * (action[i] - mean[i]) / (2 * variance)
    }
    return total
}

/// A small seedable RNG so rollout workers get independent, reproducible
/// action-sampling streams instead of contending on a shared global RNG.
public struct SplitMix64: RandomNumberGenerator {
    private var state: UInt64

    public init(seed: UInt64) {
        self.state = seed
    }

    public mutating func next() -> UInt64 {
        state &+= 0x9E37_79B9_7F4A_7C15
        var z = state
        z = (z ^ (z >> 30)) &* 0xBF58_476D_1CE4_E5B9
        z = (z ^ (z >> 27)) &* 0x94D0_49BB_1331_11EB
        return z ^ (z >> 31)
    }
}

public func sampleGaussian(mean: [Float], std: [Float], using rng: inout SplitMix64) -> [Float] {
    precondition(mean.count == std.count)
    return zip(mean, std).map { m, s in
        guard s > 0 else { return m }
        // Box-Muller.
        let u1 = Float.random(in: Float.ulpOfOne..<1, using: &rng)
        let u2 = Float.random(in: 0..<1, using: &rng)
        let z = sqrt(-2 * log(u1)) * cos(2 * Float.pi * u2)
        return m + s * z
    }
}

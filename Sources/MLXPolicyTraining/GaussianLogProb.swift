// Sources/MLXPolicyTraining/GaussianLogProb.swift
import MLX

/// MLX-array Gaussian log-probability, summed across the action-dimension
/// axis into one joint log-probability per row — the array counterpart of
/// `MuJoCoRLEnv`'s scalar `gaussianLogProb`, used inside both trainers'
/// gradient-tracked loss closures so the formula lives in exactly one place.
public func gaussianLogProbMLX(actions: MLXArray, mean: MLXArray, std: MLXArray) -> MLXArray {
    let variance = square(std)
    let perDimension = -0.5 * log(2 * Float.pi * variance) - square(actions - mean) / (2 * variance)
    return perDimension.sum(axis: -1, keepDims: true)
}

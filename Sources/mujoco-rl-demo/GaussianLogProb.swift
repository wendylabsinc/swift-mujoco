// Sources/mujoco-rl-demo/GaussianLogProb.swift
import MLX

/// MLX-array Gaussian log-probability — the array counterpart of
/// `MuJoCoRLEnv`'s scalar `gaussianLogProb`, used inside both trainers'
/// gradient-tracked loss closures so the formula lives in exactly one place.
func gaussianLogProbMLX(actions: MLXArray, mean: MLXArray, std: MLXArray) -> MLXArray {
    let variance = square(std)
    return -0.5 * log(2 * Float.pi * variance) - square(actions - mean) / (2 * variance)
}

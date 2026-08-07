// Sources/go2-locomotion-demo/main.swift
//
// Wires RobotKit + MuJoCoRLEnv + MLXPolicyTraining + Go2Kit into a runnable
// demo: `--learn` trains a Go2 locomotion policy via PPO and checkpoints it
// to disk; without `--learn`, it loads the checkpoint and runs the policy
// live, streaming the sim into the Sim tab via the same
// WorldSimServerCore/WorldSimRecorder machinery mujoco-live-demo uses.
import Foundation
import MuJoCo
import MuJoCoRLEnv
import MLXPolicyTraining
import Go2Kit
import RobotKit
import WendyMuJoCo

let observationDimensions = 45
let actionDimensions = 12
let hiddenDimensions = 64
let gamma: Float = 0.99
let episodesPerBatch = 16
let iterations = 500
let maxStepsPerEpisode = 1000
let checkpointURL = URL(fileURLWithPath: "go2-policy-checkpoint.json")

/// Adapts `Go2Environment`'s `Go2Command`-typed action interface to the
/// plain `[Float]` interface `collectEpisode`/`collectBatch` require (an MLX
/// policy network's forward pass produces a flat float vector, one residual
/// per joint — `Go2Command`'s `Double` fields exist for Go2Kit's own
/// physical-unit clarity, not for this RL plumbing). Delegates everything
/// else straight through to a private `Go2Environment`.
struct Go2VectorEnvironment: Environment {
    private var environment = Go2Environment()

    mutating func reset() -> Go2Observation { environment.reset() }

    mutating func act(_ action: [Float]) -> Go2Observation {
        environment.act(Go2Command(jointPositionResiduals: action.map(Double.init)))
    }

    var isTerminated: Bool { environment.isTerminated }
}

/// Negative squared tracking error between the robot's actual base velocity
/// (linear x/y, plus yaw rate) and the commanded `vx, vy, wz`, plus a small
/// per-step alive bonus. `Go2Environment()`'s default velocity command is
/// zero, so during training this rewards standing still and penalizes any
/// drift — the same "stay upright" behavior Task 5's own
/// zero-command-stays-upright test already validates, just now shaped into a
/// scalar reward signal a policy can be trained against.
func go2Reward(_ obs: Go2Observation) -> Float {
    let vxError = Float(obs.baseLinearVelocity.0 - obs.velocityCommand.vx)
    let vyError = Float(obs.baseLinearVelocity.1 - obs.velocityCommand.vy)
    let wzError = Float(obs.baseAngularVelocity.2 - obs.velocityCommand.wz)
    let trackingPenalty = vxError * vxError + vyError * vyError + wzError * wzError
    let aliveBonus: Float = 0.1
    return aliveBonus - trackingPenalty
}

func meanReturn(_ trajectories: [Trajectory]) -> Float {
    let totals = trajectories.map { $0.rewards.reduce(0, +) }
    return totals.reduce(0, +) / Float(totals.count)
}

func saveCheckpoint(_ weights: PolicyWeights) throws {
    struct Checkpoint: Codable {
        let w1: [Float], b1: [Float], w2: [Float], b2: [Float], logStd: [Float]
        let inputDimensions: Int, hiddenDimensions: Int, actionDimensions: Int
    }
    let checkpoint = Checkpoint(
        w1: weights.w1, b1: weights.b1, w2: weights.w2, b2: weights.b2, logStd: weights.logStd,
        inputDimensions: weights.inputDimensions, hiddenDimensions: weights.hiddenDimensions,
        actionDimensions: weights.actionDimensions
    )
    try JSONEncoder().encode(checkpoint).write(to: checkpointURL)
}

func loadCheckpoint() throws -> PolicyWeights {
    struct Checkpoint: Codable {
        let w1: [Float], b1: [Float], w2: [Float], b2: [Float], logStd: [Float]
        let inputDimensions: Int, hiddenDimensions: Int, actionDimensions: Int
    }
    let checkpoint = try JSONDecoder().decode(Checkpoint.self, from: Data(contentsOf: checkpointURL))
    return PolicyWeights(
        w1: checkpoint.w1, b1: checkpoint.b1, w2: checkpoint.w2, b2: checkpoint.b2, logStd: checkpoint.logStd,
        inputDimensions: checkpoint.inputDimensions, hiddenDimensions: checkpoint.hiddenDimensions,
        actionDimensions: checkpoint.actionDimensions
    )
}

let learn = CommandLine.arguments.contains("--learn")

await RunModeKey.$current.withValue(learn ? .learn : .infer) {
    switch RunModeKey.current {
    case .learn:
        let trainer = PPOTrainer(
            observationDimensions: observationDimensions, hiddenDimensions: hiddenDimensions,
            actionDimensions: actionDimensions, policyLearningRate: 3e-4, valueLearningRate: 1e-3,
            gamma: gamma, clipEpsilon: 0.2, epochs: 4
        )
        print("Training Go2 locomotion via PPO (\(iterations) iterations, \(episodesPerBatch) episodes/batch)")
        for iteration in 0..<iterations {
            let weights = trainer.policy.snapshot()
            let trajectories = await collectBatch(
                makeEnvironment: { Go2VectorEnvironment() }, weights: weights, episodeCount: episodesPerBatch,
                reward: go2Reward, maxSteps: maxStepsPerEpisode, baseSeed: UInt64(iteration) &* 1_000_003
            )
            let (policyLoss, valueLoss) = trainer.trainStep(trajectories: trajectories)
            print("iter \(iteration): mean return \(meanReturn(trajectories))  (policy loss \(policyLoss), value loss \(valueLoss))")
            if iteration % 20 == 0 {
                try? saveCheckpoint(trainer.policy.snapshot())
            }
        }
        try? saveCheckpoint(trainer.policy.snapshot())
    case .infer:
        guard let weights = try? loadCheckpoint() else {
            print("No checkpoint at \(checkpointURL.path) — run with --learn first.")
            return
        }
        let env = Go2Environment()
        var recorder = WorldSimRecorder()
        var observation = env.reset()
        var frame = 0
        let stepNanos: UInt64 = 20_000_000   // 50 Hz, matching go2.robot.json's policy.rate_hz
        while true {
            let (mean, _) = policyForward(weights, observation: observation.asArray)
            let command = Go2Command(jointPositionResiduals: mean.map { Double($0) })
            observation = env.act(command)
            recorder.record(
                model: env.mjModel, data: env.mjData,
                title: "go2-locomotion-demo: Go2 quadruped inference", frame: frame
            )
            frame += 1
            if env.isTerminated {
                print("fell at frame \(frame), resetting")
                observation = env.reset()
            }
            try? await Task.sleep(nanoseconds: stepNanos)
        }
    }
}

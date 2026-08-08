// Sources/RobotKitGo2/Go2JointMap.swift
import RobotKit

/// The two joint orderings this robot involves, and the permutations between
/// them.
///
/// The MuJoCo Menagerie model declares legs FL, FR, RL, RR; Unitree firmware
/// indexes motors FR, FL, RR, RL. Confusing the two silently drives every
/// command into the wrong leg, so both orders are named explicitly here and
/// the canonical order used everywhere in `RobotKit` is the firmware's.
public enum Go2JointMap {
    public static let unitreeOrder = JointMap(names: [
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    ])

    public static let mujocoOrder = JointMap(names: [
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    ])

    /// `mujocoToUnitree[i]` is the MuJoCo index of canonical (Unitree) joint `i`.
    public static let mujocoToUnitree: [Int] = mujocoOrder.permutation(to: unitreeOrder)

    /// `unitreeToMuJoCo[i]` is the canonical (Unitree) index of MuJoCo joint `i`.
    public static let unitreeToMuJoCo: [Int] = unitreeOrder.permutation(to: mujocoOrder)

    /// The Menagerie `home` keyframe pose (hip 0, thigh 0.9, calf -1.8 per leg),
    /// expressed in canonical order. Symmetric across legs, so the ordering does
    /// not change the values — it is written per-leg anyway to stay correct if
    /// an asymmetric pose is ever substituted.
    public static let defaultStandPose: [Double] = [
        0.0, 0.9, -1.8,  // FR
        0.0, 0.9, -1.8,  // FL
        0.0, 0.9, -1.8,  // RR
        0.0, 0.9, -1.8,  // RL
    ]

    /// Foot geom names in `go2.xml`, in canonical (Unitree) leg order.
    public static let footGeomNames = ["FR", "FL", "RR", "RL"]
}

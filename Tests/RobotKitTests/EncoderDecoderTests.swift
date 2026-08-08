import Testing

@testable import RobotKit

private let defaultPose: [Double] = [
    0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8, 0.0, 0.9, -1.8,
]

@Test func permutationMapsMuJoCoOrderToUnitreeOrder() {
    // MuJoCo tree order vs. Unitree firmware order.
    let mujoco = JointMap(names: [
        "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf",
        "RL_hip", "RL_thigh", "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
    ])
    let unitree = JointMap(names: [
        "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
        "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf",
    ])
    // permutation[i] is the index in `mujoco` of the joint at index i of `unitree`.
    let permutation = mujoco.permutation(to: unitree)
    #expect(permutation == [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
}

@Test func encoderProducesFortyFiveValuesInDocumentedLayout() {
    var encoder = ObservationEncoder(defaultPose: defaultPose, jointCount: 12)
    let obs = RobotObservation(
        stamp: RobotTime(nanoseconds: 0),
        joints: (0..<12).map { i in
            JointReading(position: defaultPose[i] + 0.1, velocity: 0.5, effort: 0)
        },
        imu: IMUReading(
            orientation: (1, 0, 0, 0),  // upright
            angularVelocity: (0.01, 0.02, 0.03),
            linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 10, inContact: true), count: 4)
    )
    let v = encoder.encode(obs, commandedVelocity: (0.4, 0, 0.2))

    #expect(v.count == ObservationEncoder.observationSize)
    // [0..<3] base angular velocity
    #expect(abs(v[0] - 0.01) < 1e-6)
    #expect(abs(v[2] - 0.03) < 1e-6)
    // [3..<6] projected gravity: upright means gravity points along -z in body frame
    #expect(abs(v[3] - 0) < 1e-6)
    #expect(abs(v[4] - 0) < 1e-6)
    #expect(abs(v[5] - (-1)) < 1e-6)
    // [6..<9] commanded velocity
    #expect(abs(v[6] - 0.4) < 1e-6)
    #expect(abs(v[8] - 0.2) < 1e-6)
    // [9..<21] joint position relative to default pose
    #expect(abs(v[9] - 0.1) < 1e-6)
    #expect(abs(v[20] - 0.1) < 1e-6)
    // [21..<33] joint velocity
    #expect(abs(v[21] - 0.5) < 1e-6)
    // [33..<45] previous action, zero before any action is recorded
    #expect(abs(v[33] - 0) < 1e-6)
    #expect(abs(v[44] - 0) < 1e-6)
}

@Test func encoderFeedsBackThePreviousAction() {
    var encoder = ObservationEncoder(defaultPose: defaultPose, jointCount: 12)
    let obs = RobotObservation(
        stamp: RobotTime(nanoseconds: 0),
        joints: (0..<12).map { i in JointReading(position: defaultPose[i], velocity: 0, effort: 0) },
        imu: IMUReading(
            orientation: (1, 0, 0, 0), angularVelocity: (0, 0, 0), linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 0, inContact: false), count: 4)
    )
    encoder.noteAction(Array(repeating: 0.25, count: 12))
    let v = encoder.encode(obs, commandedVelocity: (0, 0, 0))
    #expect(abs(v[33] - 0.25) < 1e-6)
    #expect(abs(v[44] - 0.25) < 1e-6)

    encoder.reset()
    let afterReset = encoder.encode(obs, commandedVelocity: (0, 0, 0))
    #expect(abs(afterReset[33] - 0) < 1e-6)
}

@Test func projectedGravityRotatesWithOrientation() {
    var encoder = ObservationEncoder(defaultPose: defaultPose, jointCount: 12)
    // 90° roll about x: (w, x, y, z) = (cos45°, sin45°, 0, 0).
    let s = (2.0).squareRoot() / 2
    let obs = RobotObservation(
        stamp: RobotTime(nanoseconds: 0),
        joints: (0..<12).map { i in JointReading(position: defaultPose[i], velocity: 0, effort: 0) },
        imu: IMUReading(
            orientation: (s, s, 0, 0), angularVelocity: (0, 0, 0), linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 0, inContact: false), count: 4)
    )
    let v = encoder.encode(obs, commandedVelocity: (0, 0, 0))
    // World -z rotated into a body rolled 90° about x becomes -y in body frame.
    #expect(abs(v[3] - 0) < 1e-5)
    #expect(abs(v[4] - (-1)) < 1e-5)
    #expect(abs(v[5] - 0) < 1e-5)
}

@Test func decoderScalesActionAroundDefaultPoseAndAppliesGains() {
    let decoder = ActionDecoder(defaultPose: defaultPose, scale: 0.25, kp: 20, kd: 0.5)
    let action = [Float](repeating: 1.0, count: 12)
    let cmd = decoder.decode(action, stamp: RobotTime(nanoseconds: 42))

    #expect(cmd.stamp.nanoseconds == 42)
    #expect(cmd.joints.count == 12)
    #expect(abs(cmd.joints[0].position - (defaultPose[0] + 0.25)) < 1e-9)
    #expect(abs(cmd.joints[1].position - (defaultPose[1] + 0.25)) < 1e-9)
    #expect(cmd.joints[0].kp == 20)
    #expect(cmd.joints[0].kd == 0.5)
    #expect(cmd.joints[0].velocity == 0)
    #expect(cmd.joints[0].feedforwardTorque == 0)
}

@Test func decoderWithZeroActionHoldsTheDefaultPose() {
    let decoder = ActionDecoder(defaultPose: defaultPose, scale: 0.25, kp: 20, kd: 0.5)
    let cmd = decoder.decode([Float](repeating: 0, count: 12), stamp: RobotTime(nanoseconds: 0))
    for i in 0..<12 {
        #expect(abs(cmd.joints[i].position - defaultPose[i]) < 1e-9)
    }
}

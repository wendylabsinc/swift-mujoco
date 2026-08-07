import Testing

@testable import RobotKit

@Test func robotTimeConvertsBetweenNanosecondsAndSeconds() {
    let t = RobotTime(nanoseconds: 1_500_000_000)
    #expect(t.seconds == 1.5)
    #expect(RobotTime(seconds: 2.25).nanoseconds == 2_250_000_000)
}

@Test func observationCarriesTwelveJointsAndFourContacts() {
    let obs = RobotObservation(
        stamp: RobotTime(nanoseconds: 0),
        joints: Array(repeating: JointReading(position: 0.1, velocity: 0.2, effort: 0.3), count: 12),
        imu: IMUReading(
            orientation: (1, 0, 0, 0), angularVelocity: (0, 0, 0), linearAcceleration: (0, 0, -9.81)),
        contacts: Array(repeating: ContactReading(normalForce: 12.0, inContact: true), count: 4)
    )
    #expect(obs.joints.count == 12)
    #expect(obs.contacts.count == 4)
    #expect(obs.joints[0].position == 0.1)
    #expect(obs.imu.linearAcceleration.2 == -9.81)
}

@Test func commandDefaultsToZeroGains() {
    let target = JointTarget(position: 0.5)
    #expect(target.velocity == 0)
    #expect(target.feedforwardTorque == 0)
    #expect(target.kp == 0)
    #expect(target.kd == 0)
}

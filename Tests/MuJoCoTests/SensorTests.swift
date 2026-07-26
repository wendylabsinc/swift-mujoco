import Testing
@testable import MuJoCo

@Test func sensorMetadataAndKinds() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    #expect(m.nsensor == 5)

    let acc = try #require(m.sensor(named: "acc"))
    #expect(acc.kind == .accelerometer)
    #expect(acc.dim == 3)

    #expect(m.sensor(named: "gyr")?.kind == .gyro)
    #expect(m.sensor(named: "fq")?.kind == .frameQuat)
    #expect(m.sensor(named: "fq")?.dim == 4)
    #expect(m.sensor(named: "rf")?.kind == .rangefinder)
    #expect(m.sensor(named: "rf")?.dim == 1)
    #expect(m.sensor(named: "tch")?.kind == .touch)
    #expect(m.sensor(named: "tch")?.dim == 1)
    #expect(m.sensor(named: "nope") == nil)

    // adr must be the running sum of dims, and nsensordata their total.
    #expect(m.nsensordata == m.sensors.reduce(0) { $0 + $1.dim })
}

@Test func sensorValuesOnGravityCompensatedBody() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    let d = MjData(m)
    mjForward(m, d)   // sensors are computed by mj_sensorPos/Vel/Acc inside mj_forward

    #expect(d.sensordata.count == m.nsensordata)

    // `probe` is a free body with gravcomp="1": the gravity-cancelling applied
    // force leaves qacc == 0 while body_weldid != 0, so the accelerometer
    // reads +gravity in the site frame instead of the 0 a jointless
    // (body_weldid == 0) body would give — see the fuller explanation on
    // Fixtures.sensorScene. Site is unrotated, so that is +9.81 on z.
    let acc = d.sensorValues(try #require(m.sensor(named: "acc")))
    #expect(acc.count == 3)
    #expect(abs(acc[0]) < 1e-6)
    #expect(abs(acc[1]) < 1e-6)
    #expect(abs(acc[2] - 9.81) < 0.05)

    // Nothing is moving.
    let gyr = d.sensorValues(try #require(m.sensor(named: "gyr")))
    #expect(gyr.allSatisfy { abs($0) < 1e-9 })

    // Body is unrotated -> identity quaternion (w,x,y,z).
    let fq = d.sensorValues(try #require(m.sensor(named: "fq")))
    #expect(abs(fq[0] - 1.0) < 1e-9)
    #expect(fq[1...3].allSatisfy { abs($0) < 1e-9 })

    // Site sits 0.5 above the floor with its z axis pointing down.
    let rf = d.sensorValues(try #require(m.sensor(named: "rf")))
    #expect(rf[0] > 0.4 && rf[0] < 0.6)

    // Nothing is touching the probe.
    let tch = d.sensorValues(try #require(m.sensor(named: "tch")))
    #expect(tch.count == 1)
    #expect(abs(tch[0]) < 1e-9)
}

@Test func withSensorValuesMatchesAllocatingForm() throws {
    let m = try MjModel.load(xml: Fixtures.sensorScene)
    let d = MjData(m)
    mjForward(m, d)
    let info = try #require(m.sensor(named: "acc"))
    let copied = d.sensorValues(info)
    let viewed = d.withSensorValues(info) { Array($0) }
    #expect(copied == viewed)
}

@Test func unknownSensorTypeMapsToOther() {
    // mjSENS_CLOCK is mapped; a deliberately out-of-range raw value must not trap.
    #expect(SensorKind(raw: 9999) == .other(9999))
}

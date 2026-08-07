import Testing

@testable import MuJoCoRLEnv

@Test func resetStartsUprightAndCentered() {
    let env = CartpoleEnv()
    let obs = env.reset()
    #expect(obs.cartPosition == 0)
    #expect(obs.poleAngle == 0)
    #expect(obs.cartVelocity == 0)
    #expect(obs.poleAngularVelocity == 0)
}

@Test func zeroActionKeepsPoleUprightBriefly() {
    let env = CartpoleEnv()
    _ = env.reset()
    let (obs, reward, done) = env.step(action: 0)
    #expect(reward == 1.0)
    #expect(done == false)
    #expect(abs(obs.poleAngle) < 0.05)
}

@Test func sustainedLargeActionEventuallyTerminates() {
    let env = CartpoleEnv()
    _ = env.reset()
    var done = false
    var steps = 0
    while !done && steps < CartpoleEnv.maxSteps {
        let (_, _, isDone) = env.step(action: 1.0)
        done = isDone
        steps += 1
    }
    #expect(done == true)
    #expect(steps < CartpoleEnv.maxSteps)
}

@Test func episodeTerminatesAtMaxStepsUnderZeroAction() {
    let env = CartpoleEnv()
    _ = env.reset()
    var done = false
    var steps = 0
    while !done {
        let (_, _, isDone) = env.step(action: 0)
        done = isDone
        steps += 1
    }
    #expect(steps == CartpoleEnv.maxSteps)
}

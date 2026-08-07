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
    let obs = env.act([0])
    #expect(env.isTerminated == false)
    #expect(abs(obs.poleAngle) < 0.05)
}

@Test func sustainedLargeActionEventuallyTerminates() {
    let env = CartpoleEnv()
    _ = env.reset()
    var steps = 0
    while !env.isTerminated && steps < CartpoleEnv.maxSteps {
        _ = env.act([1.0])
        steps += 1
    }
    #expect(env.isTerminated == true)
    #expect(steps < CartpoleEnv.maxSteps)
}

@Test func episodeTerminatesAtMaxStepsUnderZeroAction() {
    let env = CartpoleEnv()
    _ = env.reset()
    var steps = 0
    while !env.isTerminated {
        _ = env.act([0])
        steps += 1
    }
    #expect(steps == CartpoleEnv.maxSteps)
}

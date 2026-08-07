import Testing
// @testable so the pre-warmed-cache assertions can see MjModel's internal
// cachedJoints/cachedActuators/... storage.
@testable import MuJoCo
@testable import MuJoCoRLEnv

// CartpoleEnv used to call MjModel.load(xml:) per instance, which writes a
// UUID-named temp file, runs mj_loadXML, and deletes it. collectEpisode builds a
// fresh env per episode, so a 200-iteration x 16-episode run compiled the model
// 3,200 times to simulate 500 steps each — compilation dominated the physics.
// The compiled model is now shared process-wide, with its lazy introspection
// caches pre-warmed on one thread so parallel rollouts only ever read it.

@Test func allEnvsShareOneCompiledModel() {
    let a = CartpoleEnv()
    let b = CartpoleEnv()
    #expect(a.modelForTesting === b.modelForTesting)
    #expect(a.modelForTesting === CartpoleEnv.sharedModelForTesting)
}

@Test func eachEnvKeepsItsOwnData() {
    // The model is shared; the mutable per-episode state must not be.
    let a = CartpoleEnv()
    let b = CartpoleEnv()
    #expect(a.dataForTesting !== b.dataForTesting)

    _ = a.reset()
    _ = b.reset()
    for _ in 0..<25 { _ = a.step(action: 1.0) }
    #expect(a.dataForTesting.time > 0)
    #expect(b.dataForTesting.time == 0, "stepping one env must not advance another")
}

@Test func sharedModelIntrospectionCachesArePreWarmed() {
    // The caches are what make sharing safe across rollout tasks: they mutate on
    // first read, so they must already be populated before any env can see the
    // model. If this regresses, parallel rollouts race.
    let m = CartpoleEnv.sharedModelForTesting
    #expect(m.cachedJoints != nil, "joints cache must be pre-warmed")
    #expect(m.cachedActuators != nil, "actuators cache must be pre-warmed")
    #expect(m.cachedSensors != nil, "sensors cache must be pre-warmed")
    #expect(m.cachedBodyNames != nil, "bodyNames cache must be pre-warmed")
}

@Test func sharedModelHasTheExpectedCartpoleTopology() {
    let m = CartpoleEnv.sharedModelForTesting
    #expect(m.nq == 2)        // slider + hinge
    #expect(m.nv == 2)
    #expect(m.nu == 1)        // one motor on the slider
    #expect(m.id(of: objJoint, name: "slider") == 0)
    #expect(m.id(of: objJoint, name: "hinge") == 1)
}

@Test func resetIsIndependentPerEnvDespiteTheSharedModel() {
    let a = CartpoleEnv()
    let b = CartpoleEnv()
    _ = a.reset()
    for _ in 0..<40 { _ = a.step(action: 0.5) }
    let advanced = a.dataForTesting.time
    _ = b.reset()
    #expect(a.dataForTesting.time == advanced, "resetting b must not touch a")
}

@Test func parallelEpisodesProduceIndependentTrajectories() async {
    // The end-to-end shape of the sharing: many concurrent rollouts against one
    // compiled model. Distinct seeds must still give distinct action streams.
    let weights = PolicyWeights(
        w1: [Float](repeating: 0.1, count: 4 * 8),
        b1: [Float](repeating: 0, count: 8),
        w2: [Float](repeating: 0.1, count: 8),
        b2: [0],
        logStd: -0.5,
        inputDimensions: 4,
        hiddenDimensions: 8)
    let batch = await collectBatch(weights: weights, episodeCount: 8, baseSeed: 1234)
    #expect(batch.count == 8)
    for t in batch {
        #expect(!t.actions.isEmpty)
        #expect(t.actions.count == t.rewards.count)
        #expect(t.actions.count == t.observations.count)
        #expect(t.actions.count == t.logProbs.count)
    }
    // Independent seeds -> different first actions (with overwhelming probability).
    let firstActions = Set(batch.map { $0.actions.first ?? 0 })
    #expect(firstActions.count > 1, "distinct seeds must give distinct action streams")
}

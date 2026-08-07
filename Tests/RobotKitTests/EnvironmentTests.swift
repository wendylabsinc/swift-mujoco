import Testing
@testable import RobotKit

@Test func runModeDefaultsToInfer() {
    #expect(RunModeKey.current == .infer)
}

@Test func runModeScopesToWithValue() {
    #expect(RunModeKey.current == .infer)
    RunModeKey.$current.withValue(.learn) {
        #expect(RunModeKey.current == .learn)
    }
    #expect(RunModeKey.current == .infer)
}

private struct CountingEnvironment: Environment {
    var count = 0
    var isTerminated: Bool { count >= 3 }
    mutating func reset() -> Int { count = 0; return count }
    mutating func act(_ action: Int) -> Int { count += action; return count }
}

@Test func environmentConformancePlainWalkthrough() {
    var env = CountingEnvironment()
    #expect(env.reset() == 0)
    #expect(env.act(1) == 1)
    #expect(env.isTerminated == false)
    #expect(env.act(2) == 3)
    #expect(env.isTerminated == true)
}

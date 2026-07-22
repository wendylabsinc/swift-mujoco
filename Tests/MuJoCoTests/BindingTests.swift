import Testing
@testable import MuJoCo

@Test func versionStringIsNonEmpty() {
    let v = mujocoVersion()
    #expect(!v.isEmpty)
}

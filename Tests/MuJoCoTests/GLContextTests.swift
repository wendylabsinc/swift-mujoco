import Testing
@testable import MuJoCo
import CMuJoCoGL

@Test func glBackendNameIsReported() {
    // Always answers, even where no GL exists — it names the compiled-in backend.
    let name = String(cString: wmj_gl_backend_name())
    #expect(name == "egl" || name == "cgl" || name == "none")
}

@Test func glContextCreationEitherWorksOrExplainsItself() {
    if let ctx = wmj_gl_create() {
        defer { wmj_gl_destroy(ctx) }
        #expect(wmj_gl_make_current(ctx) == 1)
    } else {
        // Failure MUST come with a non-empty diagnostic naming what was missing.
        let err = String(cString: wmj_gl_last_error())
        #expect(!err.isEmpty, "wmj_gl_create returned NULL without setting an error")
    }
}

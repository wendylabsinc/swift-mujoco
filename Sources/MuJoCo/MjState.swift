import CMuJoCo

extension MjData {
    // mj_stateSize/mj_getState/mj_setState take a signed `int sig` in this MuJoCo version;
    // mjSTATE_FULLPHYSICS (a plain C enum) imports into Swift as a struct with a UInt32 rawValue.
    private var fullSpec: Int32 { Int32(mjSTATE_FULLPHYSICS.rawValue) }

    /// Number of doubles in a `mjSTATE_FULLPHYSICS` state vector for this model.
    ///
    /// Exposed so callers can size their own buffers and validate a state vector
    /// (e.g. one read off the wire, or saved from a different model) *before*
    /// handing it to ``setFullState(_:)``.
    public var fullStateSize: Int { Int(mj_stateSize(model.ptr, fullSpec)) }

    public func getFullState() -> [Double] {
        var arr = [Double](repeating: 0, count: fullStateSize)
        mj_getState(model.ptr, ptr, &arr, fullSpec)
        return arr
    }

    /// Restore a full-physics state previously produced by ``getFullState()``.
    ///
    /// - Precondition: `state.count == fullStateSize`. `mj_setState` reads
    ///   exactly `fullStateSize` doubles unconditionally, so a short array —
    ///   a truncated transfer, or a state captured from a *different* model —
    ///   would otherwise make it read past the end of the Swift buffer. This
    ///   traps instead, matching ``MjData/setCtrl(_:)-8gqf5``.
    public func setFullState(_ state: [Double]) {
        let n = fullStateSize
        precondition(state.count == n,
                     "setFullState: expected \(n) values for this model, got \(state.count)")
        var s = state
        mj_setState(model.ptr, ptr, &s, fullSpec)
        mj_forward(model.ptr, ptr)
    }
}

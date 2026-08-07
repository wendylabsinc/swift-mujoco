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

    /// Full-physics state, freshly allocated. Convenience form — see
    /// ``readFullState(into:)`` for the loop.
    public func getFullState() -> [Double] {
        var arr = [Double](repeating: 0, count: fullStateSize)
        mj_getState(model.ptr, ptr, &arr, fullSpec)
        return arr
    }

    /// Read the full-physics state into caller-owned storage.
    ///
    /// `getFullState()` allocates a fresh array per call, which an MPC or
    /// trajectory-optimisation loop does at control rate — it saves and restores
    /// state every iteration, often many times per step for a rollout. Hoist one
    /// buffer out of the loop and reuse it:
    ///
    /// ```swift
    /// var scratch = [Double](repeating: 0, count: data.fullStateSize)
    /// for _ in 0..<horizon {
    ///     data.readFullState(into: &scratch)
    ///     // …roll out, evaluate, then rewind…
    ///     data.setFullState(scratch)
    /// }
    /// ```
    ///
    /// - Precondition: `buffer.count == fullStateSize`. Same reasoning as
    ///   ``setFullState(_:)`` — `mj_getState` writes exactly that many doubles.
    public func readFullState(into buffer: inout [Double]) {
        let n = fullStateSize
        precondition(buffer.count == n,
                     "readFullState: buffer must hold \(n) values for this model, got \(buffer.count)")
        buffer.withUnsafeMutableBufferPointer { dst in
            mj_getState(model.ptr, ptr, dst.baseAddress, fullSpec)
        }
    }

    /// Restore a full-physics state from a borrowed buffer, without requiring a
    /// `[Double]`. Pairs with ``readFullState(into:)`` when the state lives in
    /// caller-owned storage that is not an `Array`.
    ///
    /// - Precondition: `buffer.count == fullStateSize`.
    public func setFullState(_ buffer: UnsafeBufferPointer<Double>) {
        let n = fullStateSize
        precondition(buffer.count == n,
                     "setFullState: expected \(n) values for this model, got \(buffer.count)")
        // mj_setState takes a non-const pointer in this MuJoCo version even though
        // it only reads; the cast is safe because it does not mutate the source.
        mj_setState(model.ptr, ptr, UnsafeMutablePointer(mutating: buffer.baseAddress), fullSpec)
        mj_forward(model.ptr, ptr)
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

import CMuJoCo

// `Span`-returning accessors for the hot-path `mjData` vectors.
//
// These sit *alongside* the `with*<R>(_ body:)` closure accessors rather than
// replacing them, deliberately. A `Span` is strictly better where it applies —
// the compiler enforces that the borrow cannot outlive `self`, whereas a closure
// accessor hands you an `UnsafeBufferPointer` you are merely *asked* not to
// escape — and it composes without nesting one closure per vector.
//
// The reason the closures stay is that producing a `Span` from a class wrapping a
// raw C pointer currently rests on three unstable things in Swift 6.3:
//
//   1. the `Lifetimes` experimental feature, enabled on this target in Package.swift
//   2. the underscored `@_lifetime(borrow self)` attribute
//   3. `_overrideLifetime`, a stdlib-internal escape hatch, needed because the
//      pointer these spans cover lives *behind* `self` rather than inside it, so
//      the compiler cannot infer the dependency on its own
//
// If any of those three change spelling, this one file stops compiling and can be
// deleted without touching the rest of the library. Making the library's primary
// hot-path API depend on them would not be a trade worth making; offering it as an
// additional, safer option is.
//
// Note that *callers* need none of the above — consuming a `Span` requires no flag.
//
// `Span` is not a `Collection` in 6.3: there is no `map`/`reduce`/`first` and no
// `Array(span)`. Iterate with `for i in span.indices`.
extension MjData {
    /// Wraps a `mjData` vector, tying the span's lifetime to a borrow of `self`.
    ///
    /// `_overrideLifetime` is doing the load-bearing work: `Span(_unsafeStart:count:)`
    /// infers its lifetime from the *pointer argument*, which is a local read out of
    /// `ptr.pointee`, so without the override the compiler correctly rejects the
    /// result as "lifetime-dependent value escapes its scope". The override asserts
    /// what is actually true — the buffer is owned by the `mjData` that `self` owns
    /// and freed in its `deinit`, so it outlives any borrow of `self`.
    @_lifetime(borrow self)
    private func vectorSpan(_ base: UnsafeMutablePointer<Double>?, _ count: Int) -> Span<Double> {
        guard let base, count > 0 else {
            return _overrideLifetime(Span<Double>(), borrowing: self)
        }
        return _overrideLifetime(Span(_unsafeStart: base, count: count), borrowing: self)
    }

    /// Generalized positions, as a borrowed view. `nq` elements.
    ///
    /// The `Span` counterpart of ``withQpos(_:)``; the allocating ``qpos`` builds a
    /// fresh Array per access.
    public var qposSpan: Span<Double> {
        @_lifetime(borrow self) get { vectorSpan(ptr.pointee.qpos, model.nq) }
    }

    /// Generalized velocities, as a borrowed view. `nv` elements.
    public var qvelSpan: Span<Double> {
        @_lifetime(borrow self) get { vectorSpan(ptr.pointee.qvel, model.nv) }
    }

    /// Generalized accelerations, as a borrowed view. `nv` elements.
    public var qaccSpan: Span<Double> {
        @_lifetime(borrow self) get { vectorSpan(ptr.pointee.qacc, model.nv) }
    }

    /// Actuator controls, as a borrowed view. `nu` elements.
    public var ctrlSpan: Span<Double> {
        @_lifetime(borrow self) get { vectorSpan(ptr.pointee.ctrl, model.nu) }
    }

    /// Every sensor's value concatenated in `adr` order, as a borrowed view.
    /// `nsensordata` elements.
    public var sensordataSpan: Span<Double> {
        @_lifetime(borrow self) get { vectorSpan(ptr.pointee.sensordata, model.nsensordata) }
    }

    /// One sensor's values, as a borrowed view sliced out by its `adr` and `dim`.
    ///
    /// The `Span` counterpart of ``withSensorValues(_:_:)``. The buffer points
    /// straight into `mjData`, so the next `mj_step` overwrites it — the borrow
    /// checker stops it escaping, but it does not stop the physics moving under it.
    @_lifetime(borrow self)
    public func sensorValuesSpan(_ info: MjModel.SensorInfo) -> Span<Double> {
        precondition(info.adr >= 0 && info.adr + info.dim <= model.nsensordata,
                     "sensorValuesSpan: sensor \"\(info.name)\" range \(info.adr)..<\(info.adr + info.dim) exceeds nsensordata \(model.nsensordata)")
        guard let base = ptr.pointee.sensordata, info.dim > 0 else {
            return _overrideLifetime(Span<Double>(), borrowing: self)
        }
        return _overrideLifetime(Span(_unsafeStart: base + info.adr, count: info.dim),
                                 borrowing: self)
    }
}

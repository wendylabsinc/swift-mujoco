import CMuJoCo

/// Typed view of MuJoCo's `mjtSensor` so callers `switch` instead of comparing
/// raw integers. `.other` keeps an unrecognized sensor representable rather
/// than trapping — MuJoCo adds sensor types between minor versions.
public enum SensorKind: Equatable, Sendable {
    case touch, accelerometer, velocimeter, gyro, force, torque, magnetometer
    case rangefinder, camProjection
    case jointPos, jointVel, actuatorPos, actuatorVel, actuatorFrc
    case framePos, frameQuat, frameXAxis, frameYAxis, frameZAxis
    case frameLinVel, frameAngVel, frameLinAcc, frameAngAcc
    case subtreeCom, subtreeLinVel, subtreeAngMom
    case clock
    case other(Int32)

    public init(raw: Int32) {
        switch raw {
        case Int32(mjSENS_TOUCH.rawValue):         self = .touch
        case Int32(mjSENS_ACCELEROMETER.rawValue): self = .accelerometer
        case Int32(mjSENS_VELOCIMETER.rawValue):   self = .velocimeter
        case Int32(mjSENS_GYRO.rawValue):          self = .gyro
        case Int32(mjSENS_FORCE.rawValue):         self = .force
        case Int32(mjSENS_TORQUE.rawValue):        self = .torque
        case Int32(mjSENS_MAGNETOMETER.rawValue):  self = .magnetometer
        case Int32(mjSENS_RANGEFINDER.rawValue):   self = .rangefinder
        case Int32(mjSENS_CAMPROJECTION.rawValue): self = .camProjection
        case Int32(mjSENS_JOINTPOS.rawValue):      self = .jointPos
        case Int32(mjSENS_JOINTVEL.rawValue):      self = .jointVel
        case Int32(mjSENS_ACTUATORPOS.rawValue):   self = .actuatorPos
        case Int32(mjSENS_ACTUATORVEL.rawValue):   self = .actuatorVel
        case Int32(mjSENS_ACTUATORFRC.rawValue):   self = .actuatorFrc
        case Int32(mjSENS_FRAMEPOS.rawValue):      self = .framePos
        case Int32(mjSENS_FRAMEQUAT.rawValue):     self = .frameQuat
        case Int32(mjSENS_FRAMEXAXIS.rawValue):    self = .frameXAxis
        case Int32(mjSENS_FRAMEYAXIS.rawValue):    self = .frameYAxis
        case Int32(mjSENS_FRAMEZAXIS.rawValue):    self = .frameZAxis
        case Int32(mjSENS_FRAMELINVEL.rawValue):   self = .frameLinVel
        case Int32(mjSENS_FRAMEANGVEL.rawValue):   self = .frameAngVel
        case Int32(mjSENS_FRAMELINACC.rawValue):   self = .frameLinAcc
        case Int32(mjSENS_FRAMEANGACC.rawValue):   self = .frameAngAcc
        case Int32(mjSENS_SUBTREECOM.rawValue):    self = .subtreeCom
        case Int32(mjSENS_SUBTREELINVEL.rawValue): self = .subtreeLinVel
        case Int32(mjSENS_SUBTREEANGMOM.rawValue): self = .subtreeAngMom
        case Int32(mjSENS_CLOCK.rawValue):         self = .clock
        default:                                   self = .other(raw)
        }
    }
}

extension MjModel.SensorInfo {
    /// Typed sensor kind. `type` is the raw `mjtSensor` value MuJoCo stores.
    public var kind: SensorKind { SensorKind(raw: Int32(type)) }
}

extension MjModel {
    /// Total length of `mjData.sensordata` — the sum of every sensor's `dim`.
    public var nsensordata: Int { Int(ptr.pointee.nsensordata) }

    /// Look up one sensor by name. Returns nil when no sensor has that name.
    public func sensor(named name: String) -> SensorInfo? {
        guard let i = id(of: objSensor, name: name) else { return nil }
        return sensors.first { $0.id == i }
    }

    /// Object type a sensor is attached to (`mjOBJ_SITE`, `mjOBJ_BODY`, …).
    public func sensorObjType(_ i: Int) -> mjtObj {
        precondition(i >= 0 && i < nsensor)
        return mjtObj(rawValue: UInt32(ptr.pointee.sensor_objtype[i]))
    }

    /// Id of the object a sensor is attached to, within `sensorObjType(i)`.
    public func sensorObjId(_ i: Int) -> Int {
        precondition(i >= 0 && i < nsensor)
        return Int(ptr.pointee.sensor_objid[i])
    }

    /// Declared noise stddev for a sensor (0 when the MJCF sets none).
    public func sensorNoise(_ i: Int) -> Double {
        precondition(i >= 0 && i < nsensor)
        return ptr.pointee.sensor_noise[i]
    }

    /// Declared cutoff for a sensor (0 means unlimited).
    public func sensorCutoff(_ i: Int) -> Double {
        precondition(i >= 0 && i < nsensor)
        return ptr.pointee.sensor_cutoff[i]
    }
}

extension MjData {
    /// Every sensor's value, concatenated in `adr` order. Allocates — prefer
    /// `withSensorValues` on the hot path.
    public var sensordata: [Double] {
        let n = model.nsensordata
        guard let base = ptr.pointee.sensordata, n > 0 else { return [] }
        return Array(UnsafeBufferPointer(start: base, count: n))
    }

    /// One sensor's values, sliced out by its `adr` and `dim`. Allocates.
    public func sensorValues(_ info: MjModel.SensorInfo) -> [Double] {
        withSensorValues(info) { Array($0) }
    }

    /// Non-allocating view of one sensor's values. This is the 200 Hz path.
    ///
    /// The buffer is only valid for the duration of `body` — it points straight
    /// into `mjData`, which the next `mj_step` overwrites.
    public func withSensorValues<R>(
        _ info: MjModel.SensorInfo,
        _ body: (UnsafeBufferPointer<Double>) -> R
    ) -> R {
        precondition(info.adr >= 0 && info.adr + info.dim <= model.nsensordata,
                     "withSensorValues: sensor \"\(info.name)\" range \(info.adr)..<\(info.adr + info.dim) exceeds nsensordata \(model.nsensordata)")
        guard let base = ptr.pointee.sensordata else {
            return body(UnsafeBufferPointer(start: nil, count: 0))
        }
        return body(UnsafeBufferPointer(start: base + info.adr, count: info.dim))
    }
}

/// Recompute position-dependent sensors without a full `mj_forward`.
public func mjSensorPos(_ m: MjModel, _ d: MjData) { mj_sensorPos(m.ptr, d.ptr) }
/// Recompute velocity-dependent sensors.
public func mjSensorVel(_ m: MjModel, _ d: MjData) { mj_sensorVel(m.ptr, d.ptr) }
/// Recompute acceleration/force-dependent sensors.
public func mjSensorAcc(_ m: MjModel, _ d: MjData) { mj_sensorAcc(m.ptr, d.ptr) }

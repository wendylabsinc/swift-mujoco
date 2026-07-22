import CMuJoCo

public final class MjData {
    public let ptr: UnsafeMutablePointer<mjData>
    let model: MjModel   // keep the model alive for this data's lifetime

    public init(_ model: MjModel) {
        self.model = model
        self.ptr = mj_makeData(model.ptr)
    }
    deinit { mj_deleteData(ptr) }

    public var time: Double { ptr.pointee.time }

    private func buffer(_ base: UnsafeMutablePointer<mjtNum>?, _ n: Int) -> [Double] {
        guard let base, n > 0 else { return [] }
        return Array(UnsafeBufferPointer(start: base, count: n))
    }

    public var qpos: [Double] { buffer(ptr.pointee.qpos, model.nq) }
    public var qvel: [Double] { buffer(ptr.pointee.qvel, model.nv) }
    public var ctrl: [Double] { buffer(ptr.pointee.ctrl, model.nu) }

    public func setCtrl(_ index: Int, _ value: Double) {
        precondition(index >= 0 && index < model.nu)
        ptr.pointee.ctrl[index] = value
    }
    public func setCtrl(_ values: [Double]) {
        for i in 0..<min(values.count, model.nu) { ptr.pointee.ctrl[i] = values[i] }
    }
}

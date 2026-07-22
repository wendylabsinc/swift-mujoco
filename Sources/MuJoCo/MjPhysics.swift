import CMuJoCo

public func mjStep(_ m: MjModel, _ d: MjData) { mj_step(m.ptr, d.ptr) }
public func mjForward(_ m: MjModel, _ d: MjData) { mj_forward(m.ptr, d.ptr) }
public func mjResetData(_ m: MjModel, _ d: MjData) { mj_resetData(m.ptr, d.ptr) }
public func mjResetDataKeyframe(_ m: MjModel, _ d: MjData, _ key: Int) {
    mj_resetDataKeyframe(m.ptr, d.ptr, Int32(key))
}

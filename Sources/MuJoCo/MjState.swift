import CMuJoCo

extension MjData {
    // mj_stateSize/mj_getState/mj_setState take a signed `int sig` in this MuJoCo version;
    // mjSTATE_FULLPHYSICS (a plain C enum) imports into Swift as a struct with a UInt32 rawValue.
    private var fullSpec: Int32 { Int32(mjSTATE_FULLPHYSICS.rawValue) }

    public func getFullState() -> [Double] {
        let n = Int(mj_stateSize(model.ptr, fullSpec))
        var arr = [Double](repeating: 0, count: n)
        mj_getState(model.ptr, ptr, &arr, fullSpec)
        return arr
    }
    public func setFullState(_ state: [Double]) {
        var s = state
        mj_setState(model.ptr, ptr, &s, fullSpec)
        mj_forward(model.ptr, ptr)
    }
}

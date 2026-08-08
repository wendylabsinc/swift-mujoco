// Sources/go2-demo/main.swift
import Foundation
import RobotKit
import RobotKitGo2
import RobotKitROS2
import RobotKitSim
import SwiftROS2

// Same controller, same encoder, same decoder, same adapter in both modes.
// Only where LowState comes from and where LowCmd goes differs.

let arguments = Array(CommandLine.arguments.dropFirst())
var mode = "sim"
var steps = 2000
var index = 0
while index < arguments.count {
    switch arguments[index] {
    case "--mode": index += 1; if index < arguments.count { mode = arguments[index] }
    case "--steps": index += 1; if index < arguments.count { steps = Int(arguments[index]) ?? steps }
    default: break
    }
    index += 1
}

let searchDirs = [
    FileManager.default.currentDirectoryPath + "/.cache/mujoco_menagerie",
    NSHomeDirectory() + "/.cache/mujoco_menagerie",
]

func makeSimulator() throws -> Go2Simulator {
    if let scene = Go2SimModel.resolveScene(searchDirs: searchDirs) {
        let wrapper = try Go2SimModel.writeWrapper(besideScene: scene)
        print("[go2-demo] model: \(wrapper)")
        return try Go2Simulator(modelXMLPath: wrapper)
    }
    print("""
        [go2-demo] Menagerie Go2 not found in \(searchDirs).
        Fetch it with:
          git clone --depth 1 --filter=blob:none --sparse \\
            https://github.com/google-deepmind/mujoco_menagerie .cache/mujoco_menagerie
          git -C .cache/mujoco_menagerie sparse-checkout add unitree_go2
        """)
    exit(2)
}

func makeRuntime() -> RobotRuntime<StandController> {
    RobotRuntime(
        controller: StandController(),
        encoder: ObservationEncoder(defaultPose: Go2JointMap.defaultStandPose, jointCount: 12),
        decoder: ActionDecoder(
            defaultPose: Go2JointMap.defaultStandPose, scale: 0.25, kp: 20, kd: 0.5),
        commandedVelocity: (0, 0, 0))
}

let adapter = Go2Adapter()

switch mode {
case "sim":
    // In-process: the simulator's LowState goes straight to the adapter.
    let sim = try makeSimulator()
    sim.reset()
    var runtime = makeRuntime()
    var command = adapter.lowCmd(
        from: runtime.tick(observation: adapter.observation(
            from: sim.lowState(), stamp: RobotTime(seconds: sim.time))))

    for step in 0..<steps {
        if step % Go2Simulator.controlDecimation == 0 {
            let observation = adapter.observation(
                from: sim.lowState(), stamp: RobotTime(seconds: sim.time))
            command = adapter.lowCmd(from: runtime.tick(observation: observation))
            if step % 500 == 0 {
                let height = observation.joints[1].position
                print("[sim] t=\(String(format: "%.2f", sim.time))s FR_thigh=\(String(format: "%.3f", height)) contacts=\(observation.contacts.filter(\.inContact).count)/4")
            }
        }
        sim.applyLowCmd(command)
        sim.step()
    }
    print("[sim] done after \(String(format: "%.2f", sim.time))s")

case "loopback":
    // Over real DDS: the simulator publishes genuine unitree_go messages on
    // /lowstate and consumes /lowcmd, exactly as hardware does. The control
    // half below never learns that its peer is a simulator.
    let sim = try makeSimulator()
    sim.reset()

    let simTransport = try await ROS2Transport(
        nodeName: "go2_sim", transport: .ddsMulticast(domainId: 0))
    let controlTransport = try await ROS2Transport(
        nodeName: "go2_control", transport: .ddsMulticast(domainId: 0))

    let commands = try await simTransport.subscribe(LowCmd.self, topic: "lowcmd")
    let states = try await controlTransport.subscribe(LowState.self, topic: "lowstate")
    try await Task.sleep(for: .milliseconds(500))  // let DDS discovery settle

    /// Holds the most recent command from the wire. An actor because the
    /// reader task and the simulation loop are different concurrency domains —
    /// a plain `var` would be a data race that Swift 6 rejects.
    actor LatestCommand {
        private var value: LowCmd?
        private var received = 0
        func store(_ cmd: LowCmd) {
            value = cmd
            received += 1
        }
        func current() -> LowCmd? { value }
        func count() -> Int { received }
    }
    let latest = LatestCommand()

    // Control side: state in, command out. Byte-for-byte the same pipeline as
    // the `sim` branch — only the source of LowState differs.
    let controlTask = Task {
        var runtime = makeRuntime()
        for await state in states {
            let observation = adapter.observation(
                from: state, stamp: RobotTime(seconds: Double(state.tick) / 1000.0))
            let command = adapter.lowCmd(from: runtime.tick(observation: observation))
            try? await controlTransport.publish(command, topic: "lowcmd")
        }
    }

    // Exactly one consumer of `commands` — an AsyncStream splits between
    // multiple consumers rather than fanning out.
    let commandReader = Task {
        for await cmd in commands { await latest.store(cmd) }
    }

    for step in 0..<steps {
        if step % Go2Simulator.controlDecimation == 0 {
            var state = sim.lowState()
            state.tick = UInt32(sim.time * 1000)
            try await simTransport.publish(state, topic: "lowstate")
            try await Task.sleep(for: .milliseconds(2))  // yield so the reply can land
            if step % 500 == 0 {
                let n = await latest.count()
                print("[loopback] t=\(String(format: "%.2f", sim.time))s commandsReceived=\(n)")
            }
        }
        if let cmd = await latest.current() { sim.applyLowCmd(cmd) }
        sim.step()
    }

    let totalReceived = await latest.count()
    controlTask.cancel()
    commandReader.cancel()
    await simTransport.shutdown()
    await controlTransport.shutdown()

    if totalReceived == 0 {
        print("[loopback] FAILED: no LowCmd ever arrived over DDS")
        exit(1)
    }
    print("""
        [loopback] done — \(totalReceived) commands flowed over DDS \
        for \(String(format: "%.2f", sim.time))s of simulated time
        """)

default:
    print("Unknown mode '\(mode)'. Usage: go2-demo --mode sim|loopback [--steps N]")
    exit(2)
}

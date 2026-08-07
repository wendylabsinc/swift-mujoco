import Foundation
import Hummingbird
import NIOCore

/// Builds the wendy-worldsim-server router: the same `/ctl/sim-*` and `/simslot/*.json`
/// shape the desktop-native Sim tab's `LiveSimClient` already speaks (see
/// wendy-sandbox/docs/superpowers/specs/2026-08-06-local-swift-mujoco-sim-bridge-design.md),
/// backed by `root`'s slot directories instead of a session container.
public func makeRouter(root: URL, heartbeatSeconds: TimeInterval = 5) -> Router<BasicRequestContext> {
    let router = Router()
    let titleCache = SceneTitleCache()
    let focusTracker = FocusTracker()

    router.get("/ctl/sim-running") { _, _ -> SimRunningResponse in
        let slots = liveSlots(root: root, heartbeatSeconds: heartbeatSeconds, now: Date())
        var running: [RunningSim] = []
        for slot in slots {
            let title = await titleCache.title(forSlot: slot.name, in: root.appendingPathComponent(slot.name))
            // swift-json-schema generates memberwise inits with parameters sorted alphabetically
            // by JSON key, not schema-declaration order — RunningSim(file:slot:title:) here, not
            // (slot:file:title:). See Task 4's report for why (CodeGenerator.swift sorts unconditionally).
            running.append(RunningSim(file: slot.name, slot: slot.name, title: title))
        }
        let focus = await focusTracker.resolve(liveNames: slots.map(\.name))
        return SimRunningResponse(focus: focus, running: running)
    }

    router.get("/ctl/sim-list") { _, _ -> SimListResponse in
        SimListResponse(current: nil, sims: [])
    }

    router.get("/simslot/{slot}/scene.json") { _, context in
        try slotFileResponse(root: root, context: context, fileName: "scene.json")
    }

    router.get("/simslot/{slot}/state.json") { _, context in
        try slotFileResponse(root: root, context: context, fileName: "state.json")
    }

    // No-ops in v1: nothing to "open" remotely (you start swift-mujoco processes yourself),
    // and there's no control channel back into a running process yet for stop/pause/step/reset.
    router.post("/ctl/sim-open") { _, _ -> EditedResponse<SimControlResponse> in
        EditedResponse(status: HTTPResponse.Status(code: 501),
                      response: SimControlResponse(control: nil, ok: false))
    }

    router.post("/ctl/sim-stop") { _, _ -> SimControlResponse in
        SimControlResponse(control: nil, ok: true)
    }

    router.post("/ctl/sim-cmd") { _, _ -> SimControlResponse in
        SimControlResponse(control: SimControl(paused: nil, reset: nil, step: nil), ok: true)
    }

    return router
}

private func slotFileResponse<Context: RequestContext>(root: URL, context: Context,
                                                        fileName: String) throws -> Response {
    guard let slot = context.parameters.get("slot") else { throw HTTPError(.badRequest) }
    // Hummingbird's router matches `{slot}` against a single raw, percent-encoded path
    // component with no normalization — a request for `/simslot/../scene.json` reaches this
    // handler with `slot == ".."` verbatim (confirmed against Trie+resolve.swift). Reject
    // anything that isn't a plain single path segment before it ever touches the filesystem.
    guard !slot.isEmpty, slot != ".", slot != "..", !slot.contains("/") else {
        throw HTTPError(.badRequest)
    }
    let fileURL = root.appendingPathComponent(slot).appendingPathComponent(fileName)
    guard let data = try? Data(contentsOf: fileURL) else { throw HTTPError(.notFound) }
    return Response(status: .ok, headers: [.contentType: "application/json"],
                    body: .init(byteBuffer: ByteBuffer(data: data)))
}

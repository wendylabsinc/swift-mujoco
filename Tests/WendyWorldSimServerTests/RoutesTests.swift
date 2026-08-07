import Testing
import Foundation
import Hummingbird
import HummingbirdTesting
import NIOFoundationCompat
@testable import WendyWorldSimServer

private func withTempRoot(_ body: (URL) async throws -> Void) async throws {
    let root = FileManager.default.temporaryDirectory.appendingPathComponent("routes-\(UUID().uuidString)")
    try FileManager.default.createDirectory(at: root, withIntermediateDirectories: true)
    defer { try? FileManager.default.removeItem(at: root) }
    try await body(root)
}

@Test func simRunningListsOnlyLiveSlots() async throws {
    try await withTempRoot { root in
        let dir = root.appendingPathComponent("cube", isDirectory: true)
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        try Data(#"{"title":"Falling cube"}"#.utf8).write(to: dir.appendingPathComponent("scene.json"))
        try Data("{}".utf8).write(to: dir.appendingPathComponent("state.json"))

        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-running", method: .get) { response in
                let body = try JSONDecoder().decode(SimRunningResponse.self, from: Data(buffer: response.body))
                #expect(body.running.map(\.slot) == ["cube"])
                #expect(body.running[0].title == "Falling cube")
                #expect(body.focus == "cube")
            }
        }
    }
}

@Test func simRunningIsEmptyWithNoSlots() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-running", method: .get) { response in
                let body = try JSONDecoder().decode(SimRunningResponse.self, from: Data(buffer: response.body))
                #expect(body.running.isEmpty)
                #expect(body.focus == nil)
            }
        }
    }
}

@Test func simListIsAlwaysEmpty() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-list", method: .get) { response in
                let body = try JSONDecoder().decode(SimListResponse.self, from: Data(buffer: response.body))
                #expect(body.sims.isEmpty)
                #expect(body.current == nil)
            }
        }
    }
}

@Test func sceneJsonStreamsFileBytes() async throws {
    try await withTempRoot { root in
        let dir = root.appendingPathComponent("cube", isDirectory: true)
        try FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        try Data(#"{"title":"Falling cube"}"#.utf8).write(to: dir.appendingPathComponent("scene.json"))

        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/simslot/cube/scene.json", method: .get) { response in
                #expect(response.status == .ok)
                #expect(Data(buffer: response.body) == Data(#"{"title":"Falling cube"}"#.utf8))
            }
        }
    }
}

@Test func sceneJsonReturns404ForMissingSlot() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/simslot/nope/scene.json", method: .get) { response in
                #expect(response.status == .notFound)
            }
        }
    }
}

@Test func sceneJsonRejectsPathTraversalInSlot() async throws {
    try await withTempRoot { root in
        // Plant a file just outside `root` (a sibling `scene.json`) so that if the traversal
        // guard in slotFileResponse were missing or broken, the naive
        // `root.appendingPathComponent(slot)` join for slot == ".." would resolve to `root`'s
        // parent directory and this file would be served back with a 200 instead of a 400.
        let secret = root.deletingLastPathComponent().appendingPathComponent("scene.json")
        try Data(#"{"title":"should never be served"}"#.utf8).write(to: secret)
        defer { try? FileManager.default.removeItem(at: secret) }

        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/simslot/../scene.json", method: .get) { response in
                #expect(response.status == .badRequest)
            }
        }
    }
}

@Test func simOpenIsANoOpThatReturns501() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-open", method: .post) { response in
                #expect(response.status.code == 501)
            }
        }
    }
}

@Test func simCmdIsANoOpThatReturnsOk() async throws {
    try await withTempRoot { root in
        let app = Application(router: makeRouter(root: root))
        try await app.test(.router) { client in
            try await client.execute(uri: "/ctl/sim-cmd", method: .post) { response in
                let body = try JSONDecoder().decode(SimControlResponse.self, from: Data(buffer: response.body))
                #expect(body.ok == true)
            }
        }
    }
}

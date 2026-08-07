import Foundation
import Hummingbird
import WendyMuJoCo
import WorldSimServerCore

@main
struct WendyWorldSimServer {
    static func main() async throws {
        let env = ProcessInfo.processInfo.environment
        let port = env["PORT"].flatMap(Int.init) ?? 8788
        let router = makeRouter(root: WorldSim.directory())
        let app = Application(router: router,
                              configuration: .init(address: .hostname("127.0.0.1", port: port)))
        try await app.runService()
    }
}

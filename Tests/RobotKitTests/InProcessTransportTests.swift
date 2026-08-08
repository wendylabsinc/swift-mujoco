import SwiftROS2
import Testing

@testable import RobotKit

@Test func inProcessTransportDeliversPublishedMessages() async throws {
    let transport = InProcessTransport()
    let stream = try await transport.subscribe(StringMsg.self, topic: "chatter")

    try await transport.publish(StringMsg(data: "one"), topic: "chatter")
    try await transport.publish(StringMsg(data: "two"), topic: "chatter")

    var received: [String] = []
    for await msg in stream {
        received.append(msg.data)
        if received.count == 2 { break }
    }
    #expect(received == ["one", "two"])
}

@Test func inProcessTransportIsolatesTopics() async throws {
    let transport = InProcessTransport()
    let stream = try await transport.subscribe(StringMsg.self, topic: "wanted")

    try await transport.publish(StringMsg(data: "ignored"), topic: "other")
    try await transport.publish(StringMsg(data: "kept"), topic: "wanted")

    var first: String?
    for await msg in stream {
        first = msg.data
        break
    }
    #expect(first == "kept")
}

@Test func inProcessTransportFansOutToMultipleSubscribers() async throws {
    let transport = InProcessTransport()
    let a = try await transport.subscribe(StringMsg.self, topic: "fan")
    let b = try await transport.subscribe(StringMsg.self, topic: "fan")

    try await transport.publish(StringMsg(data: "hello"), topic: "fan")

    var fromA: String?
    for await m in a {
        fromA = m.data
        break
    }
    var fromB: String?
    for await m in b {
        fromB = m.data
        break
    }
    #expect(fromA == "hello")
    #expect(fromB == "hello")
}

import RobotKit
import SwiftROS2

/// `MessageTransport` over real ROS 2 middleware.
///
/// Publishers are created lazily per topic and cached, so a control loop that
/// publishes every tick pays the setup cost once. Subscriptions are NOT
/// cached — each `subscribe` call creates a new underlying subscription, so
/// callers should subscribe once per topic and reuse the returned stream.
public actor ROS2Transport: MessageTransport {
    private let context: ROS2Context
    private let node: ROS2Node
    private var publishers: [String: Any] = [:]

    public init(
        nodeName: String, transport: TransportConfig, distro: ROS2Distro = .humble
    ) async throws {
        self.context = try await ROS2Context(transport: transport, distro: distro)
        self.node = try await context.createNode(name: nodeName)
    }

    public func publish<M: ROS2Message>(_ message: M, topic: String) async throws {
        let publisher: ROS2Publisher<M>
        if let cached = publishers[topic] as? ROS2Publisher<M> {
            publisher = cached
        } else {
            publisher = try await node.createPublisher(M.self, topic: topic, qos: .sensorData)
            publishers[topic] = publisher
        }
        try publisher.publish(message)
    }

    public func subscribe<M: ROS2Message>(
        _ type: M.Type, topic: String
    ) async throws -> AsyncStream<M> {
        let subscription = try await node.createSubscription(M.self, topic: topic, qos: .sensorData)
        return subscription.messages
    }

    public func shutdown() async {
        await context.shutdown()
    }
}

import SwiftROS2

/// In-memory transport: values are handed to subscribers directly, never
/// serialized. An actor because it owns the subscriber registry.
///
/// Messages published to a topic with no subscribers are dropped, matching
/// best-effort DDS behavior rather than queueing unboundedly.
public actor InProcessTransport: MessageTransport {
    private var continuations: [String: [(Any) -> Void]] = [:]

    public init() {}

    public func publish<M: ROS2Message>(_ message: M, topic: String) async throws {
        guard let sinks = continuations[topic] else { return }
        for sink in sinks { sink(message) }
    }

    public func subscribe<M: ROS2Message>(
        _ type: M.Type, topic: String
    ) async throws -> AsyncStream<M> {
        let (stream, continuation) = AsyncStream<M>.makeStream(
            bufferingPolicy: .bufferingNewest(100))
        continuations[topic, default: []].append { value in
            if let typed = value as? M { continuation.yield(typed) }
        }
        return stream
    }
}

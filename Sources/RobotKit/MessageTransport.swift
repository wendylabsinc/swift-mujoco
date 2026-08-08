import SwiftROS2

/// Publish/subscribe over ROS 2 message types, with the wire left unspecified.
///
/// `InProcessTransport` passes message values straight through with no
/// serialization; `ROS2Transport` serializes to CDR and puts them on DDS. The
/// message *types* are identical either way — only whether the bytes cross a
/// wire differs — which is what allows one control loop to serve both the fast
/// training path and the deployed path.
public protocol MessageTransport: Sendable {
    func publish<M: ROS2Message>(_ message: M, topic: String) async throws
    func subscribe<M: ROS2Message>(_ type: M.Type, topic: String) async throws -> AsyncStream<M>
}

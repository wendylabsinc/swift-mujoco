// Sources/RobotKit/Controller.swift

/// Whatever decides what the robot should do next. A hand-written policy in
/// Phase 1; a trained network later. It sees only the encoded observation
/// vector and returns only an action vector, so it is unaware of both the
/// robot's wire protocol and whether physics is simulated.
public protocol Controller: Sendable {
    mutating func act(observation: [Float]) -> [Float]
    mutating func reset()
}

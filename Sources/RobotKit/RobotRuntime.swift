// Sources/RobotKit/RobotRuntime.swift

/// One control tick: observation in, command out.
///
/// Holds the encoder (and therefore the action history) so the loop's warm-up
/// behavior is identical wherever it runs. Transport-free by design — the
/// caller decides where observations come from and where commands go, which is
/// the only thing that differs between simulation and hardware.
public struct RobotRuntime<C: Controller> {
    public var controller: C
    public var encoder: ObservationEncoder
    public let decoder: ActionDecoder
    public var commandedVelocity: (Double, Double, Double)

    /// The most recent encoded observation, exposed for logging and tests.
    public private(set) var lastObservationVector: [Float] = []

    public init(
        controller: C, encoder: ObservationEncoder, decoder: ActionDecoder,
        commandedVelocity: (Double, Double, Double)
    ) {
        self.controller = controller
        self.encoder = encoder
        self.decoder = decoder
        self.commandedVelocity = commandedVelocity
    }

    public mutating func tick(observation: RobotObservation) -> RobotCommand {
        let vector = encoder.encode(observation, commandedVelocity: commandedVelocity)
        lastObservationVector = vector
        let action = controller.act(observation: vector)
        encoder.noteAction(action)
        return decoder.decode(action, stamp: observation.stamp)
    }

    public mutating func reset() {
        encoder.reset()
        controller.reset()
        lastObservationVector = []
    }
}

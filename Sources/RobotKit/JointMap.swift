// Sources/RobotKit/JointMap.swift

/// An ordered set of joint names. Two maps over the same joints in different
/// orders yield a `permutation` that reindexes data between them — the
/// mechanism that keeps MuJoCo's leg order (FL, FR, RL, RR) from being
/// mistaken for the firmware's (FR, FL, RR, RL).
public struct JointMap: Sendable, Equatable {
    public let names: [String]

    public init(names: [String]) {
        precondition(Set(names).count == names.count, "JointMap names must be unique")
        self.names = names
    }

    public var count: Int { names.count }

    public func index(of name: String) -> Int? { names.firstIndex(of: name) }

    /// `result[i]` is this map's index for the joint that `other` holds at `i`.
    /// Reindex with `other_ordered = permutation.map { self_ordered[$0] }`.
    public func permutation(to other: JointMap) -> [Int] {
        precondition(count == other.count, "JointMap permutation requires equal counts")
        return other.names.map { name in
            guard let i = index(of: name) else {
                preconditionFailure("joint '\(name)' missing from source JointMap")
            }
            return i
        }
    }
}

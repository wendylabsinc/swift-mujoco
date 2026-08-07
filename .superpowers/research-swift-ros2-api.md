# swift-ros2 (youtalk/swift-ros2) API research — tag 1.3.0

All code quoted verbatim from `github.com/youtalk/swift-ros2` at tag `1.3.0` (fetched via `gh api repos/youtalk/swift-ros2/contents/<path>?ref=1.3.0`). File paths given are repo-relative.

---

## 1. `ROS2Message` protocol family

File: `Sources/SwiftROS2Messages/MessageProtocol.swift`

```swift
/// Type information for a ROS 2 message
public struct ROS2MessageTypeInfo: Sendable {
    /// ROS format type name (e.g., "sensor_msgs/msg/Imu")
    public let typeName: String

    /// Type hash for Jazzy+ (e.g., "RIHS01_..."), nil for Humble
    public let typeHash: String?

    public init(typeName: String, typeHash: String? = nil) {
        self.typeName = typeName
        self.typeHash = typeHash
    }
}

/// Type information for a ROS 2 service
public struct ROS2ServiceTypeInfo: Sendable {
    public let serviceName: String
    public let requestTypeName: String
    public let responseTypeName: String
    public let requestTypeHash: String?
    public let responseTypeHash: String?

    public init(
        serviceName: String,
        requestTypeName: String,
        responseTypeName: String,
        requestTypeHash: String? = nil,
        responseTypeHash: String? = nil
    ) { ... }
}

/// Type information for a ROS 2 action.
public struct ROS2ActionTypeInfo: Sendable {
    public let actionName: String
    public let goalTypeHash: String?
    public let resultTypeHash: String?
    public let feedbackTypeHash: String?
    public let sendGoalRequestTypeHash: String?
    public let sendGoalResponseTypeHash: String?
    public let getResultRequestTypeHash: String?
    public let getResultResponseTypeHash: String?
    public let feedbackMessageTypeHash: String?

    // Legacy 3-hash initializer (kept for 0.6.x source compat)
    public init(
        actionName: String,
        goalTypeHash: String? = nil,
        resultTypeHash: String? = nil,
        feedbackTypeHash: String? = nil
    )

    // Full initializer
    public init(
        actionName: String,
        goalTypeHash: String?,
        resultTypeHash: String?,
        feedbackTypeHash: String?,
        sendGoalRequestTypeHash: String?,
        sendGoalResponseTypeHash: String?,
        getResultRequestTypeHash: String?,
        getResultResponseTypeHash: String?,
        feedbackMessageTypeHash: String?
    )
}

// MARK: - CDR Protocols

/// Protocol for types that can be serialized to CDR format
public protocol CDREncodable {
    func encode(to encoder: CDREncoder) throws
}

/// Protocol for types that can be deserialized from CDR format
public protocol CDRDecodable {
    init(from decoder: CDRDecoder) throws
}

/// Combined CDR encoding and decoding
public typealias CDRCodable = CDREncodable & CDRDecodable

// MARK: - Message Protocols

/// Base protocol for ROS 2 message types
public protocol ROS2MessageType: Sendable {
    static var typeInfo: ROS2MessageTypeInfo { get }
}

public typealias ROS2Publishable = ROS2MessageType & CDREncodable
public typealias ROS2Subscribable = ROS2MessageType & CDRDecodable
public typealias ROS2Message = ROS2MessageType & CDRCodable

// MARK: - Service Protocol

public protocol ROS2ServiceType: Sendable {
    associatedtype Request: CDRCodable & Sendable
    associatedtype Response: CDRCodable & Sendable
    static var typeInfo: ROS2ServiceTypeInfo { get }
}

// MARK: - Action Protocol

public protocol ROS2Action: Sendable {
    associatedtype Goal: CDRCodable & Sendable
    associatedtype Result: CDRCodable & Sendable
    associatedtype Feedback: CDRCodable & Sendable
    static var typeInfo: ROS2ActionTypeInfo { get }
}

// MARK: - CDR Serialization Errors

public enum CDRSerializationError: Error, LocalizedError {
    case invalidCovarianceArraySize(expected: Int, actual: Int)
    case serializationFailed(String)
    case bufferOverflow
}
```

**`ROS2Message` is not a nominal protocol** — it's a typealias for `ROS2MessageType & CDREncodable & CDRDecodable`. A hand-written conforming type must implement:
- `static var typeInfo: ROS2MessageTypeInfo { get }`
- `func encode(to encoder: CDREncoder) throws`
- `init(from decoder: CDRDecoder) throws`
- be `Sendable` (required transitively by `ROS2MessageType`)

The concrete encoder/decoder types (`CDREncoder`, `CDRDecoder`) live in **`SwiftROS2CDR`**, not `SwiftROS2Wire`. `SwiftROS2Wire` contains only wire-format codecs (`ROS2Distro`, `TypeNameConverter`, `DDSWireCodec`, `ZenohWireCodec`) — it is unrelated to per-message CDR encode/decode.

### IMPORTANT gotcha: who writes the encapsulation header

`ROS2Publisher.publish` (see §4) writes the 4-byte XCDR v1 encapsulation header (`00 01 00 00`) itself via `encoder.writeEncapsulationHeader()` **before** calling `message.encode(to: encoder)`. Every real generated conformance (see `StringMsg`/`BoolMsg` below) does **not** call `writeEncapsulationHeader()` inside `encode(to:)`.

The README's own "Defining a custom message type" example (`## Defining a custom message type`) is **inconsistent** with this — it shows:
```swift
public func encode(to encoder: CDREncoder) throws {
    encoder.writeEncapsulationHeader()   // <- WRONG per Publisher.swift's contract
    try header.encode(to: encoder)
    encoder.writeFloat64(value)
}
```
Per the doc comment on `ROS2Publisher.publish` in `Sources/SwiftROS2/Publisher.swift`: *"Per-type `encode(to:)` implementations must therefore not call `writeEncapsulationHeader()` themselves."* Follow the generated-code pattern (no header call in `encode(to:)`), not the README snippet, or messages will get a doubled/misplaced header when published through `ROS2Publisher`.

---

## 2. `ROS2Context`

File: `Sources/SwiftROS2/Context.swift`

```swift
public final class ROS2Context: @unchecked Sendable {
    public let config: TransportConfig
    public let distro: ROS2Distro
    public let domainId: Int

    /// Create a new ROS 2 context
    public convenience init(
        transport: TransportConfig,
        distro: ROS2Distro = .jazzy,
        domainId: Int? = nil
    ) async throws

    /// Package-internal initializer for tests / custom transport sessions.
    package init(
        transport: TransportConfig,
        distro: ROS2Distro = .jazzy,
        domainId: Int? = nil,
        session: (any TransportSession)? = nil
    ) async throws

    /// Create a node in this context (pre-1.1 binary-stable shim).
    public func createNode(
        name: String,
        namespace: String = "/"
    ) async throws -> ROS2Node

    /// Create a node with explicit per-node options.
    public func createNode(
        name: String,
        namespace: String = "/",
        options: ROS2NodeOptions
    ) async throws -> ROS2Node

    /// Shutdown the context and all nodes
    public func shutdown() async

    public var sessionId: String { get }
    public var isConnected: Bool { get }
}
```

Only the `transport:` + `distro:` + `domainId:` convenience initializer is public; the `session:`-taking one is `package`-visibility (test-only, not usable from outside the package).

### `TransportType` enum (Sources/SwiftROS2Transport/TransportConfig.swift)

```swift
public enum TransportType: String, Codable, CaseIterable, Sendable {
    case zenoh
    case dds
    case rcl
}
```

Note: there is **no bare `TransportType` parameter on `ROS2Context.init`** — the `transport:` parameter is typed `TransportConfig` (a struct), constructed via static factory methods, not the enum directly.

### `TransportConfig` (Sources/SwiftROS2Transport/TransportConfig.swift) — full public surface

```swift
public struct TransportConfig: Sendable {
    public let type: TransportType
    public let domainId: Int
    public let zenohLocator: String?
    public let wireMode: ROS2Distro?
    public let connectionTimeout: TimeInterval
    public let ddsDiscoveryMode: DDSDiscoveryMode
    public let ddsUnicastPeers: [DDSPeer]
    public let ddsNetworkInterface: String?

    public static func zenoh(
        locator: String,
        domainId: Int = 0,
        wireMode: ROS2Distro? = nil,
        connectionTimeout: TimeInterval = 10.0
    ) -> TransportConfig

    public static func ddsMulticast(domainId: Int = 0) -> TransportConfig

    public static func ddsUnicast(peers: [DDSPeer], domainId: Int = 0) -> TransportConfig

    /// RCL + rmw_cyclonedds_cpp backend. Requires SWIFT_ROS2_ENABLE_RCL=1 build.
    public static func rcl(domainId: Int = 0) -> TransportConfig

    public static func rclUnicast(
        peers: [DDSPeer], domainId: Int = 0, interface: String? = nil
    ) -> TransportConfig

    public init(
        type: TransportType,
        domainId: Int = 0,
        zenohLocator: String? = nil,
        wireMode: ROS2Distro? = nil,
        connectionTimeout: TimeInterval = 10.0,
        ddsDiscoveryMode: DDSDiscoveryMode = .multicast,
        ddsUnicastPeers: [DDSPeer] = [],
        ddsNetworkInterface: String? = nil
    )

    public func validate() throws
}
```

`DDSDiscoveryMode`:
```swift
public enum DDSDiscoveryMode: String, Codable, CaseIterable, Sendable {
    case multicast
    case unicast
    case hybrid
}
```

`DDSPeer`:
```swift
public struct DDSPeer: Codable, Equatable, Sendable {
    public let address: String
    public let port: UInt16
    public init(address: String, port: UInt16 = 7400)
    public var locator: String { get }
    public static func discoveryPort(forDomain domainId: Int) -> UInt16
    public static func peer(address: String, domainId: Int) -> DDSPeer
}
```

### `ROS2Distro` enum — ALL cases (Sources/SwiftROS2Wire/ROS2Distro.swift)

```swift
public enum ROS2Distro: String, CaseIterable, Sendable {
    case humble
    case jazzy
    case kilted
    case rolling

    public var displayName: String { get }         // rawValue.capitalized
    public var supportsTypeHash: Bool { get }       // false only for .humble
    public var isLegacySchema: Bool { get }         // true only for .humble
    public enum WireGroup: String, CaseIterable, Sendable {
        case legacy = "Humble and earlier"
        case modern = "Iron and later"
        public var distros: [ROS2Distro] { get }
    }
    public var wireGroup: WireGroup { get }
    public var typeHashPlaceholder: String { get }  // "TypeHashNotSupported"
    public func formatTypeHash(_ typeHash: String?) -> String
    public var alwaysIncludeTypeHashInKey: Bool { get }
}
```

Default distro for `ROS2Context.init` is `.jazzy`.

### `ROS2NodeOptions` (Sources/SwiftROS2/ROS2NodeOptions.swift)

```swift
public struct ROS2NodeOptions: Sendable, Equatable {
    public var startParameterServices: Bool
    public init(startParameterServices: Bool = true)
    public static let `default` = ROS2NodeOptions()
}
```

---

## 3. `Node` (`ROS2Node`, Sources/SwiftROS2/Node.swift)

```swift
public final class ROS2Node: @unchecked Sendable {
    public let name: String
    public let namespace: String
    public let fullyQualifiedName: String

    // MARK: - Publisher
    public func createPublisher<M: CDREncodable & ROS2MessageType>(
        _ messageType: M.Type,
        topic: String,
        qos: QoSProfile = .sensorData
    ) async throws -> ROS2Publisher<M>

    // MARK: - Subscription
    public func createSubscription<M: CDRDecodable & ROS2MessageType>(
        _ messageType: M.Type,
        topic: String,
        qos: QoSProfile = .sensorData
    ) async throws -> ROS2Subscription<M>

    // MARK: - Service
    public func createService<S: ROS2ServiceType>(
        _ serviceType: S.Type,
        name: String,
        qos: QoSProfile = .servicesDefault,
        handler: @escaping @Sendable (S.Request) async throws -> S.Response
    ) async throws -> ROS2Service<S>

    public func createClient<S: ROS2ServiceType>(
        _ serviceType: S.Type,
        name: String,
        qos: QoSProfile = .servicesDefault
    ) async throws -> ROS2Client<S>

    // MARK: - Action
    public func createActionServer<H: ActionServerHandler>(
        _ actionType: H.Action.Type,
        name: String,
        qos: QoSProfile = .actionDefault,
        handler: H
    ) async throws -> ROS2ActionServer<H>

    public func createActionClient<A: ROS2Action>(
        _ actionType: A.Type,
        name: String,
        qos: QoSProfile = .actionDefault
    ) async throws -> ROS2ActionClient<A>

    // MARK: - Lifecycle
    public func destroy() async
}
```

There is no separate "node options" type passed to `createPublisher`/`createSubscription` — the only per-call knob is `qos: QoSProfile`, and it has a default (`.sensorData` for pub/sub, `.servicesDefault` for services, `.actionDefault` for actions). `ROS2NodeOptions` (§2) is a node-construction-time option only (`startParameterServices`), unrelated to publisher/subscription creation.

---

## 4. `Publisher` and `Subscription`

File: `Sources/SwiftROS2/Publisher.swift`

```swift
public final class ROS2Publisher<M: CDREncodable & ROS2MessageType>: @unchecked Sendable, PublisherCloseable {

    /// Publish a message (uses wall-clock timestamp, auto sequence number)
    public func publish(_ message: M) throws

    /// Publish a message with a caller-supplied source timestamp (nanoseconds since Unix epoch)
    /// and/or an explicit sequence number.
    public func publish(_ message: M, timestamp: UInt64, sequenceNumber: Int64? = nil) throws

    public var topic: String { get }
    public var isActive: Bool { get }
}
```

Internally, `publish` does:
```swift
let encoder = CDREncoder(isLegacySchema: isLegacySchema)
encoder.writeEncapsulationHeader()
try message.encode(to: encoder)
let data = encoder.getData()
...
try transportPublisher.publish(data: data, timestamp: timestamp, sequenceNumber: seq)
```
(confirming the header-ownership point in §1).

File: `Sources/SwiftROS2/Subscription.swift`

```swift
public final class ROS2Subscription<M: CDRDecodable & ROS2MessageType>: @unchecked Sendable, SubscriptionCloseable {

    /// AsyncStream of received messages
    public let messages: AsyncStream<M>

    /// Set a callback handler for received messages
    public func onMessage(_ handler: @escaping @Sendable (M) -> Void)

    /// Cancel the subscription
    public func cancel()

    public var topic: String? { get }
}
```

`messages` is exactly `AsyncStream<M>` (the stdlib type, not a custom `AsyncSequence`), created internally with `AsyncStream<M>(bufferingPolicy: .bufferingNewest(100))`. Consume it with plain `for await msg in sub.messages { ... }`, as in the Talker/Listener examples. There is also a parallel callback API (`sub.onMessage { msg in ... }`) — both can be used simultaneously since `receive(_:)` fans out to both the continuation and the callback.

---

## 5. `QoSProfile` (Sources/SwiftROS2/QoSProfile.swift)

```swift
public struct QoSProfile: Sendable, Equatable {
    public enum Reliability: Sendable, Equatable { case reliable; case bestEffort }
    public enum Durability: Sendable, Equatable { case volatile; case transientLocal }
    public enum History: Sendable, Equatable { case keepLast(Int); case keepAll }

    public let reliability: Reliability
    public let durability: Durability
    public let history: History

    public init(
        reliability: Reliability = .bestEffort,
        durability: Durability = .volatile,
        history: History = .keepLast(10)
    )

    // MARK: - Presets
    public static let sensorData      = QoSProfile(reliability: .bestEffort, durability: .volatile, history: .keepLast(10))
    public static let reliableSensor  = QoSProfile(reliability: .reliable,   durability: .volatile, history: .keepLast(10))
    public static let latched         = QoSProfile(reliability: .reliable,   durability: .transientLocal, history: .keepLast(1))
    public static let parameterEvents = QoSProfile(reliability: .reliable,   durability: .transientLocal, history: .keepLast(1000))
    public static let servicesDefault = QoSProfile(reliability: .reliable,   durability: .volatile, history: .keepLast(10))
    public static let actionDefault   = QoSProfile(reliability: .reliable,   durability: .volatile, history: .keepLast(10))
    public static let `default` = sensorData
}
```

Yes — `.sensorData` exists (and is the default QoS for `createPublisher`/`createSubscription`). `.reliable`/`.bestEffort` are NOT static `QoSProfile` presets themselves; they are cases of the nested `Reliability` enum, used like `QoSProfile(reliability: .reliable)`. Pass a `QoSProfile` value directly as the `qos:` argument, e.g.:
```swift
node.createPublisher(Imu.self, topic: "imu", qos: .reliableSensor)
node.createSubscription(Imu.self, topic: "imu", qos: QoSProfile(reliability: .reliable, history: .keepLast(5)))
```

---

## 6. `swift-ros2-gen` CLI and SwiftPM build plugin

### CLI (Sources/swift-ros2-gen/SwiftROS2GenCommand.swift)

Command name: `swift-ros2-gen`. Key options (ArgumentParser `@Option`/`@Flag`):

- `--input <package_name>=<directory>@<distro>` — repeatable. Example (README):
```bash
swift run swift-ros2-gen \
    --input "builtin_interfaces=vendor/rcl_interfaces-jazzy/builtin_interfaces@jazzy" \
    --input "std_msgs=vendor/common_interfaces-jazzy/std_msgs@jazzy" \
    --input "geometry_msgs=vendor/common_interfaces-jazzy/geometry_msgs@jazzy" \
    --input "sensor_msgs=vendor/common_interfaces-humble/sensor_msgs@humble" \
    --input "sensor_msgs=vendor/common_interfaces-jazzy/sensor_msgs@jazzy" \
    --output Sources/MyMessages/Generated
```
- `--output <dir>` — output directory root; required in emit mode, ignored with `--verify-hashes`.
- `--types <comma,list>` — allow-list of type names to emit.
- `--dry-run` — print planned writes, touch nothing.
- `--extra-import <Module>` — repeatable; injects extra `import` lines into generated files (used by the plugin to add `import SwiftROS2Messages`).
- `--verify-hashes <docker-image>` — RIHS01 hash oracle check (implies `--dry-run`).
- `--distros <comma,list>`, `--diagnose`, `--exclude-types` — verify-mode options.
- `--emit-rcl-marshalling`, `--rcl-c-output`, `--rcl-swift-output`, `--rcl-srv-types`, `--rcl-action-types`, `--rcl-registry-only-types` — RCL native marshalling emission mode (not needed for wire-path usage).

`--input` parsing (`parseInput`):
```swift
// Format: <package>=<path>@<distro>
let eqParts = raw.split(separator: "=", maxSplits: 1)
let atParts = eqParts[1].split(separator: "@", maxSplits: 1)
```
Malformed input throws `ValidationError("malformed --input '<raw>' (expected '<package>=<path>@<distro>')")` or `"...(missing '@<distro>')"`.

The generator walks `msg/`, `srv/`, and `action/` subdirectories of each package directory automatically — you don't point `--input` at the `msg/` subfolder itself, you point it at the package root.

### SwiftPM build plugin (Plugins/SwiftROS2GenPlugin/SwiftROS2GenPlugin.swift)

```swift
@main
struct SwiftROS2GenPlugin: BuildToolPlugin {
    func createBuildCommands(context: PluginContext, target: Target) async throws -> [Command] {
        let targetDir = target.directory
        let msgDir = targetDir.appending(subpath: "msg")
        ...
        let outputDir = context.pluginWorkDirectory.appending(subpath: "Generated")
        let toolPath = try context.tool(named: "swift-ros2-gen").path
        let inputSpec = "\(target.name)=\(targetDir.string)@jazzy"
        return [
            .buildCommand(
                displayName: "swift-ros2-gen \(target.name)",
                executable: toolPath,
                arguments: [
                    "--input", inputSpec,
                    "--output", outputDir.string,
                    "--extra-import", "SwiftROS2Messages",
                ],
                inputFiles: msgInputs,
                outputFiles: outputs
            )
        ]
    }
}
```

Constraints called out in the plugin source and README:
- **Only `msg/*.msg` files directly inside the target's own directory** are picked up (no recursion). `.srv`/`.action` files in the target directory are detected and produce a **build warning** ("these IDL kinds need the swift-ros2-gen CLI directly") — they are NOT processed by the plugin.
- The `--input` package name passed to the CLI is **the SwiftPM target's name**, always suffixed `@jazzy` (the plugin hard-codes distro `jazzy`, no multi-distro support through the plugin).
- The package/target name is PascalCased for the per-target output subdirectory but used **as-is** (not lowercased) as the `typeInfo.typeName` package segment. Per the PluginSmoke comment: *"the package name passed to the plugin is the *target* name (`PluginSmoke`), which the plugin pascal-cases to `Pluginsmoke`."* So a target literally named `PluginSmoke` produces `typeInfo.typeName == "Pluginsmoke/msg/Bool"`-shaped names — NOT `"plugin_smoke/msg/Bool"`. **README's stated constraint is: name the target snake_case/lowercase** (e.g. `my_msgs`) so the generated ROS package segment in `typeInfo.typeName` is a valid ROS package name — the plugin does not enforce or validate this, it will happily pascal-case a badly-named target and produce a wrong-looking (but non-crashing) type name.
- Struct/file naming: bare PascalCase type name, with `Msg` suffix only for stdlib-collision names (`Bool`, `String`, `Int`/`Int8..64`, `UInt`/`UInt8..64`, `Float`/`Float32`/`Float64`, `Double`, `Empty`).

### `Package.swift` product name for the plugin

```swift
.plugin(name: "SwiftROS2GenPlugin", targets: ["SwiftROS2GenPlugin"]),
```

### Worked example — Sources/Examples/PluginSmoke (verbatim Package.swift target declaration)

```swift
// the plugin generates a Swift wrapper for `msg/Bool.msg` at build
// time, and `main.swift` prints the resulting type's typeInfo.
.executableTarget(
    name: "PluginSmoke",
    dependencies: ["SwiftROS2CDR", "SwiftROS2Messages"],
    path: "Sources/Examples/PluginSmoke",
    exclude: ["msg"],
    plugins: [
        .plugin(name: "SwiftROS2GenPlugin")
    ]
),
```
(Note: inside the swift-ros2 package itself the plugin is referenced by name only, no `package:` argument, because it's a local target. A consumer depending on swift-ros2 as an external package must use `.plugin(name: "SwiftROS2GenPlugin", package: "swift-ros2")` — this is exactly what the README's "SwiftPM build plugin" section shows:)

```swift
.target(
    name: "my_msgs",
    dependencies: [
        .product(name: "SwiftROS2", package: "swift-ros2"),
    ],
    plugins: [
        .plugin(name: "SwiftROS2GenPlugin", package: "swift-ros2"),
    ]
)
```

`Sources/Examples/PluginSmoke/msg/Bool.msg` (verbatim):
```
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

bool data
```

`Sources/Examples/PluginSmoke/main.swift` (verbatim):
```swift
// Phase-7 smoke target: depends on SwiftROS2GenPlugin to generate a
// Swift wrapper for the local msg/Bool.msg at build time, then prints
// the resulting type's `typeInfo.typeName`. A non-empty stdout proves
// the plugin invoked the CLI and the build dependency on the generated
// source resolved correctly.
//
// Note: the package name passed to the plugin is the *target* name
// (`PluginSmoke`), which the plugin pascal-cases to `Pluginsmoke`. The
// generated struct is therefore `BoolMsg` namespaced under that
// per-target Pluginsmoke/ directory rather than under StdMsgs/. That is
// fine for a smoke check — the smoke target is not consumed by any
// other target and is excluded from any release product.

import Foundation
import SwiftROS2CDR
import SwiftROS2Messages

print("PluginSmoke: BoolMsg.typeInfo.typeName = \(BoolMsg.typeInfo.typeName)")
```

---

## 7. Platform / dependency requirements

From `Package.swift` (`swift-tools-version: 5.9`) and `README.md`:

**Minimum platforms** (`platforms:` array in `Package.swift`):
```swift
platforms: [
    .iOS(.v16),
    .macOS(.v13),
    .macCatalyst(.v16),
    .visionOS(.v1),
],
```
So **macOS 13.0** is the minimum. (iOS 16, Mac Catalyst 16, visionOS 1.)

**Products available to depend on** (base set, always present):
```swift
.library(name: "SwiftROS2CDR", targets: ["SwiftROS2CDR"]),
.library(name: "SwiftROS2Messages", targets: ["SwiftROS2Messages"]),
.library(name: "SwiftROS2Wire", targets: ["SwiftROS2Wire"]),
.library(name: "SwiftROS2Transport", targets: ["SwiftROS2Transport"]),
.library(name: "SwiftROS2Gen", targets: ["SwiftROS2Gen"]),
.executable(name: "swift-ros2-gen", targets: ["swift-ros2-gen"]),
.executable(name: "parity-tool", targets: ["parity-tool"]),
.plugin(name: "SwiftROS2GenPlugin", targets: ["SwiftROS2GenPlugin"]),
```
Conditionally added **only when `canBuildDDS` is true** (true unconditionally on Apple and Linux; conditionally on Windows via `CYCLONEDDS_DIR`; never on Android):
```swift
.library(name: "SwiftROS2", targets: ["SwiftROS2"]),
.library(name: "SwiftROS2DDS", targets: ["SwiftROS2DDS"]),
```
So **`SwiftROS2` (the high-level `ROS2Context`/`ROS2Node`/`ROS2Publisher`/`ROS2Subscription` umbrella) is the product name to depend on** for the API described in this report — available on macOS and Linux (and Windows with `CYCLONEDDS_DIR` set), but **not on Android** (Android is Zenoh-wire-only: `import SwiftROS2Zenoh` + `ZenohClient` directly, no `ROS2Context`/`ROS2Node`).

Also conditionally added when `!dropZenohWire` (true unless the opt-in zenoh-rmw RCL variant is selected):
```swift
.library(name: "SwiftROS2Zenoh", targets: ["SwiftROS2Zenoh"]),
```

**macOS**: `SwiftROS2` transitively depends on `SwiftROS2DDS` (which depends on `CDDSBridge` → `CCycloneDDS`) and `SwiftROS2Zenoh` (which depends on `CZenohBridge` → `CZenohPico`). On Apple platforms, `CCycloneDDS` and `CZenohPico` (the zenoh-pico wire family) resolve as prebuilt **`.binaryTarget` xcframeworks** downloaded from GitHub Releases:
```swift
return .binaryTarget(
    name: "CCycloneDDS",
    url: "\(releaseBaseURL)/CCycloneDDS.xcframework.zip",
    checksum: "36f30e40506b02cc994fc1f0e1f8f03c488d7bc6dc2e5aac37a919ccc9060d36"
)
```
`swift build` fetches these automatically — no CMake/local bootstrap needed on macOS. Standard consumer `Package.swift` snippet (from README):
```swift
// Package.swift
dependencies: [
    .package(url: "https://github.com/youtalk/swift-ros2.git", from: "1.0.0"),
],
targets: [
    .target(
        name: "YourApp",
        dependencies: [
            .product(name: "SwiftROS2", package: "swift-ros2"),
        ]
    ),
]
```

**Linux**: `CCycloneDDS` resolves via `.systemLibrary(name: "CCycloneDDS", path: "Sources/CCycloneDDS", pkgConfig: "CycloneDDS")` — requires a system CycloneDDS install discoverable through `pkg-config`. README setup:
```bash
sudo apt install ros-jazzy-cyclonedds        # or ros-humble / ros-rolling
git clone --recursive https://github.com/youtalk/swift-ros2.git
cd swift-ros2
bash Scripts/build-linux-deps.sh             # verifies pkg-config finds CycloneDDS
source /opt/ros/jazzy/setup.bash
export PKG_CONFIG_PATH=/opt/ros/jazzy/lib/$(uname -m)-linux-gnu/pkgconfig:$PKG_CONFIG_PATH
swift build
```
zenoh-pico (`CZenohPico`) is compiled from source on Linux (from `vendor/zenoh-pico`) via SwiftPM directly — no separate install step beyond `git clone --recursive` (submodules).

**Swift toolchain**: "Swift 5.9+ on Apple platforms; the CI matrix is unified on Swift 6.3.1" (README, `## Platforms` section footnote).

**Note**: depending on `SwiftROS2Zenoh` / `SwiftROS2DDS` directly is only needed if you want to call `ZenohClient`/`DDSClient` by name; they are linked transitively by `SwiftROS2` but not `@_exported`.

---

## 8. Complete publisher/subscriber examples (verbatim)

### `Sources/Examples/Talker/main.swift`

```swift
// Minimal std_msgs/String publisher — mirrors demo_nodes_cpp/talker.cpp.
// Publishes "Hello World: N" on /chatter at 1 Hz.
//
// Usage:
//   swift run talker zenoh [tcp/<host>:7447] [domain_id]
//   swift run talker dds   [domain_id]

import Foundation
import SwiftROS2

let args = Array(CommandLine.arguments.dropFirst())
let transportName = args.first ?? "zenoh"

let transport: TransportConfig
switch transportName {
case "zenoh":
    let locator = args.dropFirst().first ?? "tcp/127.0.0.1:7447"
    let domainId = args.dropFirst(2).first.flatMap(Int.init) ?? 0
    transport = .zenoh(locator: locator, domainId: domainId)
case "dds":
    let domainId = args.dropFirst().first.flatMap(Int.init) ?? 0
    transport = .ddsMulticast(domainId: domainId)
default:
    FileHandle.standardError.write(Data("Unknown transport '\(transportName)'. Use 'zenoh' or 'dds'.\n".utf8))
    exit(2)
}

let ctx = try await ROS2Context(transport: transport, distro: .jazzy)
let node = try await ctx.createNode(name: "talker")
let pub = try await node.createPublisher(StringMsg.self, topic: "chatter")

var count = 0
while !Task.isCancelled {
    count += 1
    let msg = StringMsg(data: "Hello World: \(count)")
    try pub.publish(msg)
    print("Publishing: '\(msg.data)'")
    try await Task.sleep(nanoseconds: 1_000_000_000)
}

await ctx.shutdown()
```

### `Sources/Examples/Listener/main.swift`

```swift
// Minimal std_msgs/String subscriber — mirrors demo_nodes_cpp/listener.cpp.
// Prints every message received on /chatter.
//
// Usage:
//   swift run listener zenoh [tcp/<host>:7447] [domain_id]
//   swift run listener dds   [domain_id]

import Foundation
import SwiftROS2

let args = Array(CommandLine.arguments.dropFirst())
let transportName = args.first ?? "zenoh"

let transport: TransportConfig
switch transportName {
case "zenoh":
    let locator = args.dropFirst().first ?? "tcp/127.0.0.1:7447"
    let domainId = args.dropFirst(2).first.flatMap(Int.init) ?? 0
    transport = .zenoh(locator: locator, domainId: domainId)
case "dds":
    let domainId = args.dropFirst().first.flatMap(Int.init) ?? 0
    transport = .ddsMulticast(domainId: domainId)
default:
    FileHandle.standardError.write(Data("Unknown transport '\(transportName)'. Use 'zenoh' or 'dds'.\n".utf8))
    exit(2)
}

let ctx = try await ROS2Context(transport: transport, distro: .jazzy)
let node = try await ctx.createNode(name: "listener")
let sub = try await node.createSubscription(StringMsg.self, topic: "chatter")

print("Listening on /chatter...")
for await msg in sub.messages {
    print("I heard: '\(msg.data)'")
}

await ctx.shutdown()
```

Both are declared in `Package.swift` as:
```swift
.executableTarget(name: "talker",   dependencies: ["SwiftROS2"], path: "Sources/Examples/Talker"),
.executableTarget(name: "listener", dependencies: ["SwiftROS2"], path: "Sources/Examples/Listener"),
```

### Reference: a real generated `ROS2Message` conformance (what hand-written types must match)

`Sources/SwiftROS2Messages/Generated/StdMsgs/StringMsg.swift` (verbatim, used by Talker/Listener above):
```swift
// Generated by swift-ros2-gen — DO NOT EDIT.
// Source: common_interfaces-jazzy/std_msgs/msg/String.msg

import SwiftROS2CDR

/// std_msgs/msg/String
public struct StringMsg: ROS2Message, Equatable, Sendable {
    public static let typeInfo = ROS2MessageTypeInfo(
        typeName: "std_msgs/msg/String",
        typeHash: "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
    )

    public var data: String

    public init(data: String = "") {
        self.data = data
    }

    public func encode(to encoder: CDREncoder) throws {
        encoder.writeString(data)
    }

    public init(from decoder: CDRDecoder) throws {
        self.data = try decoder.readString()
    }
}
```
Note `encode(to:)` does **not** call `writeEncapsulationHeader()` — confirms §1/§4.

---

## Appendix: `CDREncoder` / `CDRDecoder` concrete initializers

Sources/SwiftROS2CDR/CDREncoder.swift:
```swift
public final class CDREncoder {
    public let isLegacySchema: Bool
    public init(estimatedSize: Int = 256, isLegacySchema: Bool = false)
    public func reset()
    public func writeEncapsulationHeader()   // [0x00,0x01,0x00,0x00]
    public func writeUInt8/writeInt8/writeBool/writeUInt16/writeInt16/writeUInt32/writeInt32/writeUInt64/writeInt64/writeFloat32/writeFloat64(_:)
    public func writeString(_ value: String)
    public func writeFloat64Array/writeFloat32Array(_ array: [T])          // fixed-size, no length prefix
    public func writeFloat64Sequence/writeFloat32Sequence/writeInt32Sequence/writeUInt8Sequence(_:) // length-prefixed
    public func writeUInt8Sequence(_ data: Data)
    public func writeRawBytes(_ data: Data) / (_ bytes: [UInt8])
    public func writePadding(_ count: Int)
    public func getData() -> Data
    public var count: Int { get }
}
```

Sources/SwiftROS2CDR/CDRDecoder.swift:
```swift
public final class CDRDecoder {
    public static let maxSequenceElements: Int = 64 * 1024 * 1024
    public static let maxByteSequenceLength: Int = 256 * 1024 * 1024
    public static let maxStringLength: Int = 64 * 1024
    public let isLegacySchema: Bool

    /// Create a decoder from CDR data (validates encapsulation header)
    public init(data: Data, isLegacySchema: Bool = false) throws

    public var remainingBytes: Int { get }
    public func readUInt8/readInt8/readBool/readUInt16/readInt16/readUInt32/readInt32/readUInt64/readInt64/readFloat32/readFloat64() throws -> T
    public func readString() throws -> String
    public func readFloat64Array/readFloat32Array(count: Int) throws -> [T]
    public func readFloat64Sequence/readFloat32Sequence/readInt32Sequence() throws -> [T]
    public func readUInt8Sequence() throws -> Data
    public func readSequenceCount() throws -> Int
    public func readRawBytes(count: Int) throws -> Data
    public func skipBytes(_ count: Int) throws
}

public enum CDRDecodingError: Error, LocalizedError {
    case invalidEncapsulationHeader
    case unexpectedEndOfData(expected: Int, remaining: Int)
    case invalidStringEncoding
    case invalidArraySize(expected: Int, available: Int)
    case sequenceTooLarge(elements: UInt32, max: Int)
    case byteSequenceTooLarge(bytes: UInt32, max: Int)
    case stringTooLarge(length: UInt32, max: Int)
    case missingStringNullTerminator
}
```

`CDRDecoder.init(data:isLegacySchema:)` both validates the 4-byte encapsulation header is present (throws `.invalidEncapsulationHeader` otherwise) and consumes it — `ROS2Node.createSubscription`'s internal handler calls `CDRDecoder(data: data, isLegacySchema: isLegacy)` then `M(from: decoder)`, mirroring how `ROS2Publisher.publish` writes the header before `message.encode(to:)`.

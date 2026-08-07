// swift-tools-version: 6.3
import Foundation
import PackageDescription

var targets: [Target] = [
    .systemLibrary(name: "CMuJoCo", path: "Sources/CMuJoCo", pkgConfig: "mujoco"),
    .target(
        name: "CMuJoCoGL",
        cSettings: [
            .define("GL_SILENCE_DEPRECATION", to: "1", .when(platforms: [.macOS]))
        ],
        linkerSettings: [
            .linkedFramework("OpenGL", .when(platforms: [.macOS]))
        ]
    ),
    .target(
        name: "MuJoCo",
        dependencies: ["CMuJoCo", "CMuJoCoGL"],
        swiftSettings: [
            // Needed to *produce* a Span from our own accessors: the lifetime
            // dependency that ties the returned Span to `self` is still spelled
            // with the underscored `@_lifetime` and gated behind this feature in
            // Swift 6.3. Consuming a Span needs neither.
            .enableExperimentalFeature("Lifetimes")
        ]
    ),
    .executableTarget(name: "MujocoDemo", dependencies: ["MuJoCo", "WendyMuJoCo"], path: "Sources/mujoco-demo"),
    // CMuJoCo is a direct test dependency so the quaternion tests can compare the
    // Swift helpers against mju_subQuat/mju_negQuat/mju_rotVecQuat themselves,
    // rather than against a second Swift reimplementation of the same formula.
    .testTarget(name: "MuJoCoTests", dependencies: ["MuJoCo", "CMuJoCo"]),
    .target(name: "WendyMuJoCo", dependencies: ["MuJoCo", "CMuJoCo"]),
    .testTarget(name: "WendyMuJoCoTests", dependencies: ["WendyMuJoCo", "MuJoCo"]),
    .target(name: "MuJoCoRLEnv", dependencies: ["MuJoCo"]),
    .testTarget(name: "MuJoCoRLEnvTests", dependencies: ["MuJoCoRLEnv", "MuJoCo"]),
    .executableTarget(
        name: "WendyWorldSimServer",
        dependencies: [
            "WendyMuJoCo",
            .product(name: "Hummingbird", package: "hummingbird"),
            .product(name: "NIOCore", package: "swift-nio"),
        ],
        path: "Sources/wendy-worldsim-server",
        plugins: [.plugin(name: "JSONSchemaPlugin", package: "swift-json-schema")]
    ),
    .testTarget(
        name: "WendyWorldSimServerTests",
        dependencies: [
            "WendyWorldSimServer",
            .product(name: "HummingbirdTesting", package: "hummingbird"),
            .product(name: "NIOCore", package: "swift-nio"),
            .product(name: "NIOFoundationCompat", package: "swift-nio"),
        ]
    ),
]

var products: [Product] = [
    .library(name: "MuJoCo", targets: ["MuJoCo"]),
    .executable(name: "mujoco-demo", targets: ["MujocoDemo"]),
    .library(name: "WendyMuJoCo", targets: ["WendyMuJoCo"]),
    .executable(name: "wendy-worldsim-server", targets: ["WendyWorldSimServer"]),
]

var dependencies: [Package.Dependency] = [
    .package(url: "https://github.com/hummingbird-project/hummingbird.git", from: "2.0.0"),
    .package(url: "https://github.com/wendylabsinc/swift-json-schema.git", from: "0.1.0"),
    // Referenced directly (ByteBuffer in Routes.swift, Data(buffer:) in RoutesTests.swift) —
    // SPM requires a package to be listed here to use its products, even though Hummingbird
    // already depends on it transitively.
    .package(url: "https://github.com/apple/swift-nio.git", from: "2.60.0"),
]

// The MLX-backed RL demo is opt-in via `MUJOCO_RL_DEMO=1`, NOT `#if os(macOS)`.
//
// A manifest's `#if os(...)` describes the machine *running SwiftPM*, not the
// build target. On a Mac cross-compiling for Linux ARM64 — exactly what
// `wendy run` does when it builds a Swift app for a device — `#if os(macOS)` is
// true, so MujocoRLDemo and mlx-swift (Metal-only) were pulled into a Linux
// build graph and broke it. Linux CI never caught this because Linux CI builds
// *on* Linux, where the branch is correctly skipped.
//
// Build it with:  MUJOCO_RL_DEMO=1 swift build
let buildRLDemo = ProcessInfo.processInfo.environment["MUJOCO_RL_DEMO"] == "1"

if buildRLDemo {
// Pinned exactly rather than `from:` because this dependency is conditional:
// Package.resolved only carries pins for the *default* configuration, so a
// range here would leave the opt-in build unpinned and non-reproducible.
dependencies.append(.package(url: "https://github.com/ml-explore/mlx-swift", exact: "0.31.6"))

targets.append(
    .executableTarget(
        name: "MujocoRLDemo",
        dependencies: [
            "MuJoCoRLEnv",
            .product(name: "MLX", package: "mlx-swift"),
            .product(name: "MLXNN", package: "mlx-swift"),
            .product(name: "MLXOptimizers", package: "mlx-swift"),
        ],
        path: "Sources/mujoco-rl-demo"
    )
)
targets.append(.testTarget(name: "MujocoRLDemoTests", dependencies: ["MujocoRLDemo", "MuJoCoRLEnv"]))
products.append(.executable(name: "mujoco-rl-demo", targets: ["MujocoRLDemo"]))
}

let package = Package(
    name: "swift-mujoco",
    platforms: [.macOS(.v14)],
    products: products,
    dependencies: dependencies,
    targets: targets
)

// swift-tools-version: 6.1
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
    .target(name: "MuJoCo", dependencies: ["CMuJoCo", "CMuJoCoGL"]),
    .executableTarget(name: "MujocoDemo", dependencies: ["MuJoCo", "WendyMuJoCo"], path: "Sources/mujoco-demo"),
    .testTarget(name: "MuJoCoTests", dependencies: ["MuJoCo"]),
    .target(name: "WendyMuJoCo", dependencies: ["MuJoCo", "CMuJoCo"]),
    .testTarget(name: "WendyMuJoCoTests", dependencies: ["WendyMuJoCo", "MuJoCo"]),
    .target(name: "MuJoCoRLEnv", dependencies: ["MuJoCo"]),
    .testTarget(name: "MuJoCoRLEnvTests", dependencies: ["MuJoCoRLEnv", "MuJoCo"]),
    .target(name: "RobotKit", path: "Sources/RobotKit"),
    .testTarget(name: "RobotKitTests", dependencies: ["RobotKit"]),
    .target(
        name: "WorldSimServerCore",
        dependencies: [
            "WendyMuJoCo",
            .product(name: "Hummingbird", package: "hummingbird"),
            .product(name: "NIOCore", package: "swift-nio"),
        ],
        path: "Sources/WorldSimServerCore",
        plugins: [.plugin(name: "JSONSchemaPlugin", package: "swift-json-schema")]
    ),
    .executableTarget(
        name: "WendyWorldSimServer",
        dependencies: ["WendyMuJoCo", "WorldSimServerCore",
                      .product(name: "Hummingbird", package: "hummingbird")],
        path: "Sources/wendy-worldsim-server"
    ),
    .testTarget(
        name: "WendyWorldSimServerTests",
        dependencies: [
            "WorldSimServerCore",
            .product(name: "HummingbirdTesting", package: "hummingbird"),
            .product(name: "NIOCore", package: "swift-nio"),
            .product(name: "NIOFoundationCompat", package: "swift-nio"),
        ]
    ),
    .executableTarget(
        name: "MujocoLiveDemo",
        dependencies: ["MuJoCo", "WendyMuJoCo", "WorldSimServerCore",
                      .product(name: "Hummingbird", package: "hummingbird")],
        path: "Sources/mujoco-live-demo"
    ),
]

var products: [Product] = [
    .library(name: "MuJoCo", targets: ["MuJoCo"]),
    .executable(name: "mujoco-demo", targets: ["MujocoDemo"]),
    .library(name: "WendyMuJoCo", targets: ["WendyMuJoCo"]),
    .library(name: "WorldSimServerCore", targets: ["WorldSimServerCore"]),
    .executable(name: "wendy-worldsim-server", targets: ["WendyWorldSimServer"]),
    .executable(name: "mujoco-live-demo", targets: ["MujocoLiveDemo"]),
    .library(name: "RobotKit", targets: ["RobotKit"]),
]

var dependencies: [Package.Dependency] = [
    .package(url: "https://github.com/hummingbird-project/hummingbird.git", from: "2.0.0"),
    .package(url: "https://github.com/wendylabsinc/swift-json-schema.git", from: "0.1.0"),
    // Referenced directly (ByteBuffer in Routes.swift, Data(buffer:) in RoutesTests.swift) —
    // SPM requires a package to be listed here to use its products, even though Hummingbird
    // already depends on it transitively.
    .package(url: "https://github.com/apple/swift-nio.git", from: "2.60.0"),
]

#if os(macOS)
dependencies.append(.package(url: "https://github.com/ml-explore/mlx-swift", from: "0.31.6"))

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
#endif

let package = Package(
    name: "swift-mujoco",
    platforms: [.macOS(.v14)],
    products: products,
    dependencies: dependencies,
    targets: targets
)

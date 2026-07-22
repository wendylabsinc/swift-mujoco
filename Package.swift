// swift-tools-version: 6.1
import PackageDescription

let package = Package(
    name: "swift-mujoco",
    platforms: [.macOS(.v13)],
    products: [
        .library(name: "MuJoCo", targets: ["MuJoCo"]),
        .executable(name: "mujoco-demo", targets: ["MujocoDemo"]),
    ],
    targets: [
        .systemLibrary(name: "CMuJoCo", path: "Sources/CMuJoCo", pkgConfig: "mujoco"),
        .target(name: "MuJoCo", dependencies: ["CMuJoCo"]),
        .executableTarget(name: "MujocoDemo", dependencies: ["MuJoCo"], path: "Sources/mujoco-demo"),
        .testTarget(name: "MuJoCoTests", dependencies: ["MuJoCo"]),
    ]
)

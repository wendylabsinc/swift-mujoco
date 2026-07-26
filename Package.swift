// swift-tools-version: 6.1
import PackageDescription

let package = Package(
    name: "swift-mujoco",
    platforms: [.macOS(.v13)],
    products: [
        .library(name: "MuJoCo", targets: ["MuJoCo"]),
        .executable(name: "mujoco-demo", targets: ["MujocoDemo"]),
        .library(name: "WendyMuJoCo", targets: ["WendyMuJoCo"]),
    ],
    targets: [
        .systemLibrary(name: "CMuJoCo", path: "Sources/CMuJoCo", pkgConfig: "mujoco"),
        .target(
            name: "CMuJoCoGL",
            linkerSettings: [
                .linkedFramework("OpenGL", .when(platforms: [.macOS]))
            ]
        ),
        .target(name: "MuJoCo", dependencies: ["CMuJoCo", "CMuJoCoGL"]),
        .executableTarget(name: "MujocoDemo", dependencies: ["MuJoCo"], path: "Sources/mujoco-demo"),
        .testTarget(name: "MuJoCoTests", dependencies: ["MuJoCo"]),
        .target(name: "WendyMuJoCo", dependencies: ["MuJoCo", "CMuJoCo"]),
        .testTarget(name: "WendyMuJoCoTests", dependencies: ["WendyMuJoCo", "MuJoCo"]),
    ]
)

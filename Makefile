.PHONY: build reinforce

build:
	xcodebuild build -scheme mujoco-rl-demo -destination 'platform=macOS' -derivedDataPath .build-xcode -configuration Release

reinforce: build
	.build-xcode/Build/Products/Release/mujoco-rl-demo reinforce

# Local swift-mujoco → Sim tab bridge — design (pointer)

**Date:** 2026-08-06
**Status:** Approved (brainstorming) — pending implementation plan

Full design lives in the `wendy-sandbox` repo, since it spans both repos and
that's where the prior Sim-tab design docs already live:

`wendy-sandbox/docs/superpowers/specs/2026-08-06-local-swift-mujoco-sim-bridge-design.md`

## What lands in this repo (swift-mujoco)

- `WendyMuJoCo/WorldSim.swift`: slot-directory convention
  (`WENDY_WORLDSIM_SLOT` env var, `<root>/<slot>/{scene.json,state.json}`).
- `WendyMuJoCo/WorldSimRecorder.swift` (new): wraps `buildScene`/`buildState`
  + `WorldSim.writeAtomic` into a per-step recorder.
- `mujoco-demo`: wired to the recorder as the first live consumer.
- `wendy-worldsim-server` (new executable target): a Hummingbird HTTP server
  that serves the same `/ctl/sim-running`, `/ctl/sim-list`,
  `/simslot/<slot>/scene.json`, `/simslot/<slot>/state.json` shape the
  `desktop-native` Sim tab already speaks — backed by the slot directories on
  disk, using [swift-json-schema](https://github.com/wendylabsinc/swift-json-schema)
  for the small JSON control responses.

No changes to any MuJoCo binding or physics code — this is purely a streaming/
serving layer on top of what `WendyMuJoCo` already produces.

See the full doc (linked above) for architecture, the file-vs-in-memory
rationale, error handling, and testing.

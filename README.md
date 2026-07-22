# swift-mujoco

Swift bindings for the MuJoCo physics engine.

## Prerequisites
Install MuJoCo (headers + shared lib) from the pip wheel. The system Python may not have
a `mujoco` wheel available (e.g. Python 3.14) — use a Python 3.12 environment via `uv`:

    uv venv --python 3.12 .venv-mujoco
    uv pip install --python .venv-mujoco/bin/python mujoco
    PYTHON=.venv-mujoco/bin/python MUJOCO_PREFIX="$HOME/.local" ./Scripts/install-mujoco.sh

This installs headers/lib into `$HOME/.local` (no `sudo` required) and writes
`$HOME/.local/lib/pkgconfig/mujoco.pc`.

## Build & test
    export PKG_CONFIG_PATH=$HOME/.local/lib/pkgconfig
    swift build
    swift test

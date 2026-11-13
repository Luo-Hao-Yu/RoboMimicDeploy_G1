#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
# The existing Parkour environment contains the shared ONNX/observation runtime.
if [[ ! -x .venv-parkour/bin/python ]]; then
    echo 'Create .venv-parkour first; see SETUP_zh.md.' >&2
    exit 1
fi
if [[ ! -d .deps/unitree_sdk2_python ]]; then
    git clone https://github.com/unitreerobotics/unitree_sdk2_python.git .deps/unitree_sdk2_python
    git -C .deps/unitree_sdk2_python checkout 814556d15970dd2ecf1c9984e845ca02ab07e206
fi
# Python 3.11 has no upstream 0.10.2 wheel on this platform: build local DDS.
if [[ ! -d .deps/cyclonedds-src ]]; then
    git clone --depth 1 --branch releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds.git .deps/cyclonedds-src
fi
cmake -S .deps/cyclonedds-src -B .deps/cyclonedds-build \
    -DCMAKE_INSTALL_PREFIX="$repo_dir/.deps/cyclonedds" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DENABLE_SSL=NO -DENABLE_SECURITY=NO
cmake --build .deps/cyclonedds-build --parallel 4
cmake --install .deps/cyclonedds-build
CYCLONEDDS_HOME="$repo_dir/.deps/cyclonedds" .venv-parkour/bin/python -m pip install -r requirements-real.txt
.venv-parkour/bin/python -m pip check

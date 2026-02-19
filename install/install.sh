#!/usr/bin/env bash
set -euo pipefail

echo "=== [1/6] APT deps ==="
sudo apt update
sudo apt install -y \
  git wget curl build-essential cmake pkg-config \
  python3 python3-pip python3-venv \
  gazebo libgazebo-dev \
  libprotobuf-dev protobuf-compiler \
  libeigen3-dev \
  net-tools

echo "=== [2/6] Python deps (optional but useful) ==="
python3 -m pip install --user -U pip

echo "=== [3/6] ArduPilot (SITL) ==="
if [ ! -d "$HOME/ardupilot" ]; then
  git clone https://github.com/ArduPilot/ardupilot.git "$HOME/ardupilot"
fi

cd "$HOME/ardupilot"
git submodule update --init --recursive

# ArduPilot build deps (script oficial)
echo "=== Installing ArduPilot prereqs (may take a bit) ==="
Tools/environment_install/install-prereqs-ubuntu.sh -y

# recarga grupo (para no tener que reiniciar sesión a veces)
# shellcheck disable=SC1091
. "$HOME/.profile" || true

echo "=== [4/6] Build ArduCopter SITL once ==="
cd "$HOME/ardupilot/ArduCopter"
../waf configure --board sitl
../waf copter

echo "=== [5/6] ardupilot_gazebo (Gazebo Classic 11 plugin) ==="
if [ ! -d "$HOME/ardupilot_gazebo" ]; then
  git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$HOME/ardupilot_gazebo"
fi

cd "$HOME/ardupilot_gazebo"
git fetch --all --tags
git checkout gazebo11

rm -rf build
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"

echo "=== [6/6] Quick sanity checks ==="
echo "[OK] gazebo version:"
gazebo --version || true

echo "[OK] ArduPilotPlugin built at:"
find "$HOME/ardupilot_gazebo/build" -name "libArduPilotPlugin.so" -print || true

echo
echo "DONE"
echo "Next: create env.sh and source it before running."

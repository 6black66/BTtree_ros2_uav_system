#!/usr/bin/env bash
# Build the official Agent locally, using the installed ROS Jazzy DDS libraries.
set -eo pipefail
UAV_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cmake -S "$UAV_REPO_ROOT/thirdparty/micro_xrce_dds_agent" \
  -B "$UAV_REPO_ROOT/master_ws/build/xrce_agent" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$UAV_REPO_ROOT/master_ws/install/xrce_agent" \
  -DCMAKE_INSTALL_RPATH='$ORIGIN/../lib' \
  -DUAGENT_SUPERBUILD=OFF -DUAGENT_USE_SYSTEM_FASTDDS=ON \
  -DUAGENT_USE_SYSTEM_FASTCDR=ON -DUAGENT_USE_SYSTEM_LOGGER=ON \
  -DUAGENT_P2P_PROFILE=OFF -DUAGENT_BUILD_TESTS=OFF -DUAGENT_LOGGER_PROFILE=OFF
cmake --build "$UAV_REPO_ROOT/master_ws/build/xrce_agent" --parallel "${UAV_BUILD_JOBS:-2}"
cmake --install "$UAV_REPO_ROOT/master_ws/build/xrce_agent"

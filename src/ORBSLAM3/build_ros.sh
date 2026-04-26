#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Building ROS nodes"

ROS_PKG_DIR="Examples/ROS/ORB_SLAM3"
if [ ! -d "${ROOT_DIR}/${ROS_PKG_DIR}" ]; then
  ROS_PKG_DIR="Examples_old/ROS/ORB_SLAM3"
fi

ROS_PARENT_DIR="$(dirname "${ROS_PKG_DIR}")"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:${ROOT_DIR}/${ROS_PARENT_DIR}"

# Likewise avoid cache conflicts with the old build directory.
ROS_BUILD_DIR_NAME="${ROS_BUILD_DIR_NAME:-build_orbs}"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

cmake -S "${ROOT_DIR}/${ROS_PKG_DIR}" -B "${ROOT_DIR}/${ROS_PKG_DIR}/${ROS_BUILD_DIR_NAME}" -DROS_BUILD_TYPE=Release
cmake --build "${ROOT_DIR}/${ROS_PKG_DIR}/${ROS_BUILD_DIR_NAME}" -j "${BUILD_JOBS}"

#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Use an independent build directory by default to avoid CMakeCache conflicts with the sibling `src/ORB_SLAM3` directory.
BUILD_DIR_NAME="${BUILD_DIR_NAME:-build_orbs}"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

build_cmake_project() {
  local src_dir="$1"
  local build_dir="$2"
  local name="$3"

  echo "Configuring and building ${name} ..."
  cmake -S "${src_dir}" -B "${build_dir}" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${build_dir}" -j "${BUILD_JOBS}"
}

build_cmake_project "${ROOT_DIR}/Thirdparty/DBoW2" "${ROOT_DIR}/Thirdparty/DBoW2/${BUILD_DIR_NAME}" "Thirdparty/DBoW2"
build_cmake_project "${ROOT_DIR}/Thirdparty/g2o" "${ROOT_DIR}/Thirdparty/g2o/${BUILD_DIR_NAME}" "Thirdparty/g2o"
build_cmake_project "${ROOT_DIR}/Thirdparty/Sophus" "${ROOT_DIR}/Thirdparty/Sophus/${BUILD_DIR_NAME}" "Thirdparty/Sophus"

echo "Uncompress vocabulary ..."
if [ ! -f "${ROOT_DIR}/Vocabulary/ORBvoc.txt" ]; then
  tar -xf "${ROOT_DIR}/Vocabulary/ORBvoc.txt.tar.gz" -C "${ROOT_DIR}/Vocabulary"
fi

build_cmake_project "${ROOT_DIR}" "${ROOT_DIR}/${BUILD_DIR_NAME}" "ORB_SLAM3"

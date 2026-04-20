echo "Building ROS nodes"

# 优先使用当前仓库的 ROS 包，避免与其他同名 ORB_SLAM2 冲突
cd Examples/ROS/ORB_SLAM2
export ROS_PACKAGE_PATH="$(pwd):${ROS_PACKAGE_PATH}"

mkdir -p build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j

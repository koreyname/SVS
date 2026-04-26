echo "Building ROS nodes"

# Prefer the ROS package in the current repository to avoid conflicts with other packages named ORB_SLAM2.
cd Examples/ROS/ORB_SLAM2
export ROS_PACKAGE_PATH="$(pwd):${ROS_PACKAGE_PATH}"

mkdir -p build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j

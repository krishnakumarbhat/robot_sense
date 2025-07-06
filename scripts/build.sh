#!/bin/bash

# Source ROS environment
source /opt/ros/humble/setup.bash

# Create workspace if it doesn't exist
mkdir -p ~/realsense_actuator_project/src
cd ~/realsense_actuator_project

# Copy source code from Windows to WSL
cp -r /mnt/c/Users/zalikapope/Desktop/Ros/realsense_actuator_project/src/* src/

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build packages
colcon build

# Source the workspace
source install/setup.bash

echo "Build complete! You can now run: ros2 launch realsense_driver camera_record.launch.py"

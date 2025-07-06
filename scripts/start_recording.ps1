# Start recording script
Write-Host "Starting RealSense Camera Recording..."
Write-Host "Press 's' to start/stop recording, Ctrl+C to exit"

# Launch the ROS nodes in WSL
wsl -e bash -ic @'
source /opt/ros/humble/setup.bash
cd ~/realsense_actuator_project
source install/setup.bash
ros2 launch realsense_driver camera_record.launch.py
'@

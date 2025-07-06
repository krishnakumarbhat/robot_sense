# Install WSL if not already installed
if (!(wsl -l -v)) {
    Write-Host "Installing WSL..."
    wsl --install
}

# Install Ubuntu if not already installed (you may need to restart after this)
if (!(wsl -l -v | Select-String "Ubuntu")) {
    Write-Host "Installing Ubuntu..."
    wsl --install -d Ubuntu
}

# Instructions for the user
Write-Host @"
Please complete the following steps:

1. Open Ubuntu WSL (search for Ubuntu in Start menu)
2. Run the following commands in Ubuntu:

sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-pip
sudo apt install -y ros-dev-tools
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y ros-humble-rosbag2
sudo apt install -y libopencv-dev

# Install RealSense SDK
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -y
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# Install Python dependencies
pip3 install pyrealsense2
pip3 install opencv-python
pip3 install numpy
pip3 install keyboard

# Source ROS environment
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc

3. After installation, you can run your ROS2 nodes using:
   ros2 run <package_name> <node_name>
"@

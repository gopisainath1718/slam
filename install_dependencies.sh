#!/bin/bash

# Exit on error
set -e

echo "Updating package lists..."
sudo apt-get update

echo "Installing ROS 2 dependencies (assuming ROS 2 is already installed)..."
sudo apt-get install -y ros-dev-tools

echo "Installing Eigen3..."
sudo apt-get install -y libeigen3-dev

echo "Installing OpenCV..."
sudo apt-get install -y libopencv-dev

echo "Installing Pangolin dependencies..."
sudo apt-get install -y libgl1-mesa-dev libglew-dev

echo "Installing Pangolin from source..."
# Clone Pangolin if it doesn't exist
if [ ! -d "/tmp/Pangolin" ]; then
    git clone https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin
fi

cd /tmp/Pangolin
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install

echo "Dependencies installed successfully!"

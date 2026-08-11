#!/bin/bash

# Update package list
sudo apt-get update

# Install UR packages
sudo apt-get install -y ros-humble-ur

# Install MoveIt packages
sudo apt-get install -y ros-humble-moveit ros-humble-moveit-visual-tools ros-humble-moveit-ros-perception ros-humble-moveit-planners-chomp

# Install other packages
sudo apt-get install -y usbutils evtest cabextract dkms gedit sysfsutils

# Clean up
sudo apt-get clean

echo "Installation of UR, MoveIt packages is complete."
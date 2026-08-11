#!/bin/bash

# Update package list
sudo apt-get update

# Install PlotJuggler packages
sudo apt-get install -y ros-humble-plotjuggler-ros

# Clean up
sudo apt-get clean

echo "Installation of PlotJuggler is complete."
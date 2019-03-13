#!/bin/bash
sudo apt update && sudo apt upgrade -y
sudo apt install python-rosinstall python-rosdep
sudo rosdep init
rosdep update

cd Workspace/src
rosinstall --catkin . ../../uwrt_rover.rosinstall
rosdep install --from-paths . --ignore-src -r --rosdistro kinetic -y

echo "Dependencies installed"

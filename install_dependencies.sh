#!/bin/bash

sudo apt install -y python-pip
sudo pip install numpy

cd Workspace
wstool init ../ ../dependencies.rosinstall
rosdep install --from-paths src --ignore-src -r -y

echo "Dependencies installed"

# MarsRover2019
[![Build Status](https://travis-ci.org/uwrobotics/MarsRover2019.svg?branch=master)](https://travis-ci.org/uwrobotics/MarsRover2019)

All software for the 2019 UWRT Mars Rover 


To get all required dependencies, navigate to the project root directory and run: 
```
./install_dependencies.sh
```
Put all future dependencies in the dependencies.rosinstall file


# Simulation
Ensure that all submodules are loaded, and that the husky submodule is on the kinetic-devel branch.
Comment out the spawn_husky node in "husky/husky_gazebo/launch/spawn_husky.launch" and the GPS and IMU sensors in
"husky/husky_description/urdf/husky.urdf.xacro".

# Coding Standards
For **C++** code, we follow the ROS coding guidelines. There is a `.clang_format` file that can automatically format your C++ code in the correct style.

To run the formatter, install the package `clang-format`
```
sudo apt install clang-format
```
Once the package is installed, you can run the following command:
```
cd <folder containing repository>/Workspace/src
find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format -i -style=file $1
```

For **Python 2.7** code, we follow the PEP8 standard. Install `pycodestyle` to find any styling errors.
```
sudo pip install pycodestyle
```
To check your Python files:
```
cd <folder containing repository>/Workspace/src
find . -name '*.py' -exec pycodestyle {} \;


# MarsRover2019
[![Build Status](https://travis-ci.com/uwrobotics/MarsRover2019.svg?branch=master)](https://travis-ci.com/uwrobotics/MarsRover2019)

All software for the 2019 UWRT Mars Rover 


To get all required dependencies, navigate to the project root directory and run: 
```
./install_dependencies.sh
```
Put all future non-rosdep dependencies in the dependencies.rosinstall file

# Build
We recommend using the `catkin build` command from the `catkin_tools` package to build your workspace,  instead of using `catkin_make`. `catkin_tools` as a whole is more reliable and predictable, as it builds each package in isolation. It is also what our CI uses.

To install `catkin_tools`, run the following commands:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt update
sudo apt-get install python-catkin-tools
```

# Simulation
Ensure that all submodules are loaded, and that the husky submodule is on the kinetic-devel branch.
Comment out the spawn_husky node in "husky/husky_gazebo/launch/spawn_husky.launch" and the GPS and IMU sensors in
"husky/husky_description/urdf/husky.urdf.xacro".

# Rover Setup
The rover can be setup to start the ros software automatically when it boots. This is can be done using the robot_upstart package.


To install:
```
    $ rosrun robot_upstart install uwrt_bringup/launch/rover.launch
```

This will create a job called ``uwrt`` on your machine, which launches
base.launch. It will start automatically when you next start your machine,
or you can bring it up and down manually:


To enable/disable startup service:
```
    $ sudo service uwrt start
    $ sudo service uwrt stop
```

If the job rover is crashing on startup, view the logs:
```
    $ sudo tail /var/log/upstart/uwrt.log -n 30
```

For more details, please read robot_upstart documentation.

# Coding Standards
For **C++** code, we follow the ROS coding guidelines. There is a `.clang_format` file that can automatically format your C++ code in the correct style.

To run the formatter, install the package `clang-format`
```
sudo apt install clang-format
```
Once the package is installed, you can run the following command:
```
cd <folder containing repository>/Workspace/src/<package_youre_working_on>
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


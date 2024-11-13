# Loco-manipulation Control
This package implements an MPC-based controller for locomotion and [loco-manipulation](https://ieeexplore.ieee.org/abstract/document/10160523) on Unitree quadruped robots, including A1, Alinego, and Go1 models.

# Installation
## Prerequisites

The library is written in C++11, and it is tested under Ubuntu 20.04 with library versions as 
provided in the package sources.

## Dependencies

* C++ compiler with C++11 support
* Eigen (v3.3)
* Boost C++ (v1.71)
* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)
* ROS Noetic
* catkin ``sudo apt-get install catkin``
* pybind11_catkin, ROS package, installable via ``sudo apt install ros-noetic-pybind11-catkin``
* catkin-pkg package for python3. Install with ``sudo apt install python3-catkin-tools``


## Build the library

Create a new catkin workspace:

```
# Create the directories
# Do not forget to change <...> parts
mkdir -p <directory_to_ws>/<catkin_ws_name>/src
cd <directory_to_ws>/<catkin_ws_name>/

# Initialize the catkin workspace
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
Clone the code:

```
# Navigate to the directory of src
# Do not forget to change <...> parts
cd <directory_to_ws>/<catkin_ws_name>/src
git clone --recurse-submodules https://github.com/DRCL-USC/Loco_manipulation_control.git
```
Build simulation package:
```
cd ..
catkin build quadruped_sim
```
Build hardware package:
```
catkin build quadruped_hardware
```

# Usage

# Citation 

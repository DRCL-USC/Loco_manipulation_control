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
* ROS Noetic (For simulation only)
* catkin ``sudo apt-get install catkin``
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
cd <directory_to_ws>/<catkin_ws_name>/src

# Clone the repo:
git clone --depth 1 --recurse-submodules https://github.com/DRCL-USC/Loco_manipulation_control.git
```
Build the simulation package:
```
cd ..
catkin build quadruped_sim
```
Build the hardware package:
```
catkin build quadruped_hardware
```

# Usage
## Running Simulation

> [!NOTE]
> In any terminal you open, make sure to source the packages you have built first:
>
> ```source devel/setup.bash```

First, launch the Gazebo world:
```
roslaunch quadruped_sim world_only.launch
```
You can also create and launch your own custom world.

Next, spawn the robot into the world:
```
roslaunch quadruped_sim spawn_robot.launch robot_type:=<robot_type> ns:=<ns> x_pos:=<x> y_pos:=<y> yaw_angle:=<yaw>
```
Specify the robot type (`a1`, `alinego`, or `go1`) using `<robot_type>`, the namespace with `<ns>` (to differentiate between robots in multi-robot simulations), and the initial position and orientation of the robot with `<x>`, `<y>`, and `<yaw>`. You can spawn multiple robots by running the above command in separate terminals, ensuring each robot has a unique namespace.

Then, start the controller node for each robot:
```
rosrun quadruped_sim quad_sim_control <robot_type> __ns:=<ns>
```
Ensure you provide the correct `<robot_type>` and namespace `<ns>` for each robot. 

### Using Tmux Script for Multi-Robot Simulation

A Tmux script, `multiRobotSim.yaml`, located in the `quadruped_sim/scripts` directory, automates the process of running the simulation explained above and can work for multiple robots (the files contains two robots). To use this script, first install:

```
sudo apt-get install tmux tmuxp
```

Next, navigate to the scripts directory and load the Tmux session:

```
cd quadruped_sim/scripts
tmuxp load multiRobotSim.yaml
```
You can create a custom Tmux script for any number of robots, similar to the provided `multiRobotSim.yaml`.

### Keyboard Commands:
The robot operates in various modes:
- `Passive`: No actuation is applied to the robot.
- `PD Stand`: The robot stands using a PD controller for each joint.
- `QP Stand`: The robot stands using a [balancing QP-based](https://iit-dlslab.github.io/papers/focchi2016.pdf) controller.
- `Walking`: The robot walks using [convex MPC](https://ieeexplore.ieee.org/document/8594448).
- `Loco-manipulation`: The robot performs loco-manipulation using a [unified MPC](https://ieeexplore.ieee.org/abstract/document/10160523).

To switch between different state machines, use the following keyboard commands in the controller node terminal. The initial mode when starting the controller is `Passive`.

| Finite State Machine            | Keyboard Command |
|---------------------------------|------------------|
| Passive to PD Stand             | 2                |
| PD Stand to QP Stand            | 0                |
| QP Stand to Walking             | 4                |
| QP Stand to Loco-manipulation   | 9                |
| Any mode to Passive             | 1                |


Note that you must transition through the states consecutively to reach the `Walking` or `Loco-manipulation` modes. Directly jumping to these modes is not possible.

To control the robot's velocity in `Walking` mode, use the following keyboard commands: `w` for forward velocity, `s` for backward velocity, and `a` and `d` for positive and negative yaw rate, respectively.

In `Loco-manipulation` mode, the velocity command and manipulation force can only be controlled through ROS commands, which are explained later.

### ROS Commands:
You can also switch between different state machines by publishing the following ROS topics:
| Finite State Machine            | ROS Command                                                                 |
|---------------------------------|------------------------------------------------------------------------------|
| Passive to PD Stand             | `rostopic pub --once /<ns>/FSM std_msgs/String 'data: "PDSTAND"'`            |
| PD Stand to QP Stand            | `rostopic pub --once /<ns>/FSM std_msgs/String 'data: "QPSTAND"'`            |
| QP Stand to Walking             | `rostopic pub --once /<ns>/FSM std_msgs/String 'data: "WALKING"'`            |
| QP Stand to Loco-manipulation   | `rostopic pub --once /<ns>/FSM std_msgs/String 'data: "MANIPULATION"'`   |
| Any mode to Passive             | `rostopic pub --once /<ns>/FSM std_msgs/String 'data: "PASSIVE"'`            |

In `Loco-manipulation` mode, you can send manipulation force commands via the `/<ns>/wrench` topic and velocity commands via the `/<ns>/cmd_vel` topic.

## Running Hardware Experiment
First, build the hardware package:
```
catkin build quadruped_hardware
```
To run hardware experiments, navigate to the following directory:
```
cd devel/lib/quadruped_hardware
```
Then execute:
```
./<robot_type>_hardware_control
```
Replace `<robot_type>` with `a1`, `aliengo`, or `go1`.

You can switch modes and send velocity commands using the keyboard as described in the keyboard commands section. The default command panel is the keyboard. Alternatively, you can use the Unitree joystick to control the robot by running the following command:
```
./<robot_type>_hardware_control wireless
```
Switch between state machines using the joystick commands as follows:
| Finite State Machine            | Joystick Command |
|---------------------------------|------------------|
| Passive to PD Stand             | L2+A             |
| PD Stand to QP Stand            | L1+X             |
| QP Stand to Walking             | START            |
| QP Stand to Loco-manipulation   | L1+A             |
| Any mode to Passive             | L2+B             |

Note that the hardware code does not use ROS, so command communication via ROS is not available. For manipulation commands in `Loco-manipulation` mode, we use a UDP connection. Details and configurations for this can be modified in `ManipulationUDP.cpp`.

# Citation 
```
@inproceedings{Sombolestan2023,
    title = {{Hierarchical Adaptive Loco-manipulation Control for Quadruped Robots}},
    year = {2023},
    booktitle = {2023 IEEE International Conference on Robotics and Automation (ICRA)},
    author = {Sombolestan, Mohsen and Nguyen, Quan},
    month = {5},
    pages = {12156--12162},
    publisher = {IEEE},
    doi = {10.1109/ICRA48891.2023.10160523}
}
```
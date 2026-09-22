# Wheeltec AutoRace Simulation

ROS 2 Humble + Gazebo Fortress simulation environment for a Wheeltec autonomous vehicle.

The goal of the project is to create a simulation environment where a Wheeltec robot follows a marked 2D track using a camera-based lane-following algorithm.

## Target environment

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Fortress
- ros_gz
- OpenCV

## Packages

### wheeltec_autorace_description

Robot models, sensors, meshes and simulation assets.

### wheeltec_autorace_gazebo

Gazebo worlds used by the simulation.

### wheeltec_autorace_bringup

ROS 2 launch files and ROS-Gazebo bridge configuration.

### wheeltec_autorace_application

Camera processing and lane-following algorithms.

## Build

```bash
cd ~/wheeltec_autorace_ws

source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash

ros2 launch wheeltec_autorace_bringup simulation.launch.py

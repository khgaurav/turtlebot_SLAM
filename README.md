# ROS Sim2Real Environment for RL Experiment

This repository contains a ROS workspace for creating a sim2real environment for a Reinforcement Learning (RL) experiment. The project focuses on mapping a traffic intersection and localizing within that map using RTAB-Map with a RealSense camera.
This project specifically implements SLAM (Simultaneous Localization and Mapping) for Turtlebot2 using an Intel RealSense D435i camera on ROS Melodic. The rs_rtabmap package in the repository structure is designed to integrate the RealSense camera with RTAB-Map for 3D mapping and localization.

## Overview

The project consists of two main components:

1. **Mapping and Localization**: Using RTAB-Map for creating a 3D map of a traffic intersection and localizing the robot within it.
2. **RL Policy Execution**: Implementing an actor-critic policy to control the robot's movement.

## Prerequisites

- ROS Melodic
- Turtlebot packages
- RTAB-Map ROS
- RealSense Camera packages
- Python
- OpenCV
- NumPy

## Installation

1. Clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/khgaurav/turtlebot_SLAM.git
```

2. Install dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
```


## Usage

### Mapping and Localization

To start the mapping process:

```bash
roslaunch rs_rtabmap turtlebot_slam.launch rviz:=false
```

For localization mode (using a pre-built map):

```bash
roslaunch rs_rtabmap turtlebot_slam.launch localization:=true
```


### RL Policy Execution

To run the code to capture required data for a RL policy and convert actions from the actor-critic RL algorithm to Real Life:

```bash
rosrun rs_rtabmap sim2real.py
```

This script processes depth images and controls the robot's motion based on the RL policy.

## Key Components

### Mapping and Localization (RTAB-Map)

The `mapping.launch` file sets up RTAB-Map for 3D mapping and localization with the following features:

- RGB-D odometry
- Loop closure detection
- 2D SLAM optimization
- Integration with RealSense camera
- Move base for navigation


### RL Policy Execution

The depth processor and motion controller script implements:

- Depth image processing (resizing to 84x84 for RL input)
- Robot motion control based on RL policy output
- Conversion of wheel speeds to linear and angular velocities

# UR5 Robot Simulation Setup Guide

This guide provides step-by-step instructions to set up a simulation of a UR5 robot for ROS control using the `ure_moveit_config` package. The guide covers launching the ur5e in rviz, and using `ur_commander.py` for planning simulation to a random pose.

## Table of Contents
1. [Setup the Required Packages](#setup-the-required-packages)
<!--2. [Extract Calibration Information](#extract-calibration-information)
3. [Start the Robot Driver](#start-the-robot-driver)
4. [Using MoveIt! for Motion Planning](#using-moveit-for-motion-planning)
5. [Reference](#reference)
-->
## Setup the Required Packages

1. Clone the _universal_robots_ repository: Create a catkin workspace. Then, clone the repository within _src_ folder in the _ur5_ws_ workspace.

## Launch the RViz Simulation
```bash
roslaunch ur5e_moveit_config demo.launch
```

## Run the Commander File in Simulation Mode
```bash
rosrun image2position ur5_commander.py -m sim
```
<!-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

1. Create a configuration with Robotiq gripper attached with UR5 robot:-

	* If not done already, clone the _universal_robots_ repository: Create a catkin workspace. Then, clone the repository within _src_ folder in the _ur5_ws_ workspace.
 	* Clone the robotiq repository.
		The robotiq repository hosted on the official account of ros-industrial Github organisation does not provide support for ROS noetic. However, a modified repository with required support is provided at https://github.com/jr-robotics/robotiq. 

```
git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
```
```
git clone https://github.com/jr-robotics/robotiq.git
```
 

2. Combine the UR5 arm and the gripper using a URDF file.
3. Start a UR5 with gripper in Rviz
4. Control it with Rviz GUI as well as with terminal command line tool



References:
1. https://roboticscasual.com/ros-tutorial-how-to-create-a-moveit-config-for-the-ur5-and-a-gripper/
2. https://github.com/jr-robotics/robotiq-->


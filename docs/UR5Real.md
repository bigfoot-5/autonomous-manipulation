# UR5 Robot Setup Guide

This guide provides step-by-step instructions to set up a UR5 robot for ROS control using the `ur_robot_driver` package. The guide covers configuring the robot, extracting calibration information, launching the robot driver, and using MoveIt! for motion planning.

## Table of Contents
1. [Setup the UR5 Robot](#setup-the-ur5-robot)
2. [Extract Calibration Information](#extract-calibration-information)
3. [Start the Robot Driver](#start-the-robot-driver)
4. [Using MoveIt! for Motion Planning](#using-moveit-for-motion-planning)
5. [Reference](#reference)

## Setup the UR5 Robot

1. Power ON the UR5 robot using the teach pendant and follow these steps to configure it:
    i. On the teach pendant, navigate to **Setup Robot -> Network**:
        - Select the 'Static Address' option under 'Select your network method' and press 'Apply'.
        - Wait until the display shows ':heavy_check_mark: Network is connected'.
        - Note the IP address of the robot[[2]](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial).

    ii. Go to 'URCaps' tab and ensure that 'External Control' is displayed under 'Active URCaps':
        - Press the '+' icon.
        - Select '*.urcap' file.
        - Press 'Open'.
        - Select 'Restart' Option. After the robot restarts, continue with the next steps.

    iii. On the teach pendant, go to **Program Robot -> Installation -> External Control**:
        - Confirm the Host IP.
        - Type '50002' in the Custom port.
        - The Host name field may be left empty.

    iv. Go to 'Program' tab; press 'Empty Program'; In the left section, below 'Robot Program', press `<empty>`.

    v. In the 'Insert program lines here' section, press 'Structure', press 'URCaps' tab, and press 'External Control'.
    
2. Extract the calibration information from the robot - this shall provide the current parameters of the robot in a 'yaml' file[[5]](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc):
    ```bash
    roslaunch ur_calibration calibration_correction.launch robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
    ```

3. Start the robot driver using the existing launch file and pass the calibration information along with it:
    ```bash
    roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<robot_ip> kinematics_config:="${HOME}/my_robot_calibration.yaml"
    ```

4. On the teach pendant, press the :arrow_forward: icon on the bottom left of the last used display screen in step 1.

## Using MoveIt! for Motion Planning

5. Use [MoveIt!](http://wiki.ros.org/action/show/moveit?action=show&redirect=MoveIt) to control the robot and allow motion planning[[2]](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial):
    ```bash
    roslaunch ur5e_moveit_config moveit_planning_execution.launch
    ```
    The screen should now show - 'You can start planning now!'

6. Start [RViz](http://wiki.ros.org/rviz), including the MoveIt! motion planning plugin:
    ```bash
    roslaunch ur5e_moveit_config moveit_rviz.launch
    ```

7. Use RViz interface to set the goal state of the robot by changing the required joint angles.
8. Reduce the speed on the teach pendant to around 20% - 25%.
9. Click on 'Plan' to ensure the path is free of obstacles.
10. Click 'Execute' to verify the movement of UR5 joints.

## Reference

- [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) - Date of access: 07/07/2023
- [http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) - Date of access: 07/07/2023
- [https://github.com/ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) - Date of access: 07/07/2023
- [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md#installing-a-urcap-on-a-cb3-robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md#installing-a-urcap-on-a-cb3-robot) - Date of access: 07/07/2023
- [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc) - Date of access: 07/07/2023

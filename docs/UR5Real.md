# UR5 Robot Setup Guide

This guide provides step-by-step instructions to set up a UR5 robot for ROS control using the `ur_robot_driver` package. The guide covers configuring the robot, extracting calibration information, launching the robot driver, and using MoveIt! for motion planning.

## Table of Contents
1. [Setup the UR5 Robot](#setup-the-ur5-robot)
2. [Extract Calibration Information](#extract-calibration-information)
3. [Start the Robot Driver](#start-the-robot-driver)
4. [Using MoveIt! for Motion Planning](#using-moveit-for-motion-planning)
5. [Reference](#reference)

## Universal Robot's teach pendant
![home screen of teach pendent20230809_122650](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/39c7919b-aae4-4f4c-b3ac-25b87ac89825)
<p align="center">Home screen of the UR5 teach pendant</p>

## Setup the UR5 Robot

1. Power ON the UR5 robot using the teach pendant and follow these steps to configure it:

   i. On the teach pendant, navigate to **Setup Robot -> Network**:
![20230809_122905marked](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/484f5f27-180b-46a4-9cf9-145efe03e5f2)
<p align="center">Display after selecting 'Setup Robot'</p>

- Select the 'Static Address' option under 'Select your network method'.

![20230809_123004marked](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/88a82f4a-d567-4848-bb67-e049c4fa0e88)
<p align="center">Display after navigating to 'Network'</p>

- Once the fields in 'Network detailed settings' section are filled, tap on 'Apply'.
- Wait until the display shows ':heavy_check_mark: Network is connected'.
- Note the IP address of the robot[[2]](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial).

    ii. Tap on **Back**; Navigate to **URCaps** tab and ensure that **External Control** is displayed under **Active URCaps**:
![20230809_122928marked](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/5f6bdecf-9669-46c8-9780-a2c580d7ac6e)

If **External Control** is not displayed, copy the required '\*.urcap' file using a USB drive. Now, follow these steps to install URCaps:

- Tap the '+' icon.
![20230809_122928Plusmarked](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/d4f3e685-110e-4458-bf7a-fb6308538045)

- Navigate to the required olcation on USB Drive and select '*.urcap' file.
![20230809_122950](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/3c1d9829-7d4e-4017-8190-075ab4c2e976)

- Tap 'Open'.
- Select 'Restart' Option. After the robot restarts, continue with the next steps.

    iii. On the teach pendant home screen, go to **Program Robot -> Installation -> External Control**:

![20230809_123034marked](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/f31fb691-c301-4c13-9865-477548f15e53)

- Confirm the Host IP.
- Type '50002' in the Custom port.
- 'Host name' is not a required field. Any name may be given or it may be left empty.

    iv. Navigate to **Program -> Empty Program**
  ![20230809_123055marked](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/f80dbd24-5f31-457b-9a40-cba0fb863df5)

    v. Navigate to **Structure -> URCaps -> External Control**.
    ![20230809_123137marked](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/806b9e36-5b6c-424f-a99b-bedb1d45bc56)

3. Extract the calibration information from the robot - this shall provide the current parameters of the robot in a 'yaml' file[[5]](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc):
    ```bash
    roslaunch ur_calibration calibration_correction.launch robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
    ```

4. In a new terminal, start the robot driver using the existing launch file and pass the calibration information along with it:
    ```bash
    roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<robot_ip> kinematics_config:="${HOME}/my_robot_calibration.yaml"
    ```

5. On the teach pendant, press the :arrow_forward: icon on the last used display screen from step 2.v.
![20230809_123137Playmarked](https://github.com/robotvisionlabs/autonomous-manipulation/assets/17614773/fe9a6d7f-6f40-4868-8b12-9a28dfde95dd)

## Using MoveIt! for Motion Planning

5. In a new terminal, use [MoveIt!](http://wiki.ros.org/action/show/moveit?action=show&redirect=MoveIt) to control the robot and allow motion planning[[2]](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial):
    ```bash
    roslaunch ur5e_moveit_config moveit_planning_execution.launch
    ```
    The screen should now show - 'You can start planning now!'

6. In a new terminal, start [RViz](http://wiki.ros.org/rviz), including the MoveIt! motion planning plugin:
    ```bash
    roslaunch ur5e_moveit_config moveit_rviz.launch
    ```

7. Use RViz interface to set the goal state of the robot by changing the required joint angles. This may be done by moving a joint on the robot model to the required angle using mouse, or by moving sliders in **joints** tab to the required angle.
8. Reduce the speed on the teach pendant to around 20% - 25%.
9. Click on 'Plan' to ensure the path is free of obstacles.
10. Click 'Execute' to verify the movement of UR5 joints.

## Reference

- [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) - Date of access: 07/07/2023
- [http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) - Date of access: 07/07/2023
- [https://github.com/ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) - Date of access: 07/07/2023
- [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md#installing-a-urcap-on-a-cb3-robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md#installing-a-urcap-on-a-cb3-robot) - Date of access: 07/07/2023
- [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc) - Date of access: 07/07/2023

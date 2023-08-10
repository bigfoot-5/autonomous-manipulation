Setup the UR5 robot for ur_robot_driver:

1. Power ON the UR5 robot using the teach pendant and follow the steps below to configure it:
   
	i) On the teach pendant, navigate to the Setup Robot -> Network:
	* Under 'Select your network method', select the 'Static Address' option and press the 'Apply' icon.
	* Wait till the display shows ':heavy_check_mark:Network is connected'.
 	* Note the IP address of the robot[[2]](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial). This shall be used to establish a connection with the pc. 

	ii) Go to 'URCaps' tab and ensure that 'External Control' is displayed under 'Active URCaps'. If it is not displayed then-
	* press on the '\+' icon
 	* select '*.urcap' file
  	* press on 'Open'
   	* select 'Restart' Option. After the robot restarts, continue with the next steps.
	
	iii) On the teach pendant - go to Program Robot -> Installation -> External Control:
	* confirm the Host IP
 	* type '50002' in the Custom port
  	* the Host name field is not required to be filled, it may be left empty

	iv) Go to 'Program' tab; press 'Empty Program'; In the left section, below 'Robot Program', press <'empty'>.

	v) In the 'Insert program lines here' section, press 'Structure', press 'URCaps' tab and press 'External Control'.

Now, the left section of the display, below 'Robot Program', will show 'Control by hostname', according to the hostname which is entered in previous point. If the Host name was not entered, then it will show some random IP address.

2. Extract the calibration information from the robot - this shall provide the current parameters of the robot in a 'yaml' file[[5]](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc). Copy the following command into the terminal:
```
roslaunch ur_calibration calibration_correction.launch robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
```
3.  Start the robot driver using the existing launch file and pass the calibration information along with it. Copy the following command in a new terminal.
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<robot_ip> kinematics_config:="${HOME}/my_robot_calibration.yaml"
```
4. On the teach pendant, on the last used display screen in point no. 1, press the :arrow_forward: icon on bottom left.
5. Use [MoveIt!](http://wiki.ros.org/action/show/moveit?action=show&redirect=MoveIt) to control the robot and allow motion planning[[2]](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial). Copy the following command in a new terminal.
```
roslaunch ur5e_moveit_config moveit_planning_execution.launch
```
The screen should now show - 'You can start planning now!'

6. Start [RViz](http://wiki.ros.org/rviz), including the MoveIt! motion planning plugin. Copy the following command in a new terminal.:
```
roslaunch ur5e_moveit_config moveit_rviz.launch
```
7. Use RViz interface to fix the goal state of robot by changing the required joint angles.
8. Reduce the Speed on the teach pendant to around 20% - 25%.
9. Click on plan - ensure the path is free of obstacles.
10. Click execute - verify the movement of UR5 joints.


Reference:

[1] [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver); Date of access - 07/07/2023

[2] [http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial); Date of access - 07/07/2023

[3] [https://github.com/ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot); Date of access - 07/07/2023

[4] [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md#installing-a-urcap-on-a-cb3-robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md#installing-a-urcap-on-a-cb3-robot); Date of access - 07/07/2023

[5] [https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/tree/master#prepare-the-ros-pc); Date of access - 07/07/2023


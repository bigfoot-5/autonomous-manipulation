Setup the UR5 robot for ur_robot_driver:

1. Configure the hardware, and note the IP address of the robot[2] - The tutorial in this link is deprecated, only till step 3.2 shall be followed.
2.
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]
```
[details to be added]


<!--Setting up MoveIt![details to be added]
```
roslaunch ur5_moveit_config moveit_planning_execution.launch
```
-->

Reference:

[1]
```
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
```
[2]
```
http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial
```
[3]
```
https://github.com/ros-industrial/universal_robot
```
[4]
```
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md#installing-a-urcap-on-a-cb3-robot
```

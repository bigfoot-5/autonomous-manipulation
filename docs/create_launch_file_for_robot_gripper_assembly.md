## To create a launch file of the assembly of ur5e robot and robotiq_2f_85 gripper

# Create a new ROS package
In `ur5_ws/src` folder

```bash
catkin_create_pkg my_robot std_msgs rospy roscpp
```

```bash
cd ~/ur5_ws
catkin_make
```


```bash
source ~/ur5_ws/devel/setup.bash
```

# Copy the required files in the new package
1. Create a folder 'URDF' in 'my_robot'
2. Copy the `robotiq_arg2f_85_macro.xacro` file from `/home/mridulsongara/ur5_ws/src/robotiq/robotiq_2f_85_gripper_gazebo/urdf` folder into the `my_robot/urdf` folder.
 - The copy is required only because this file need to be modified for instantiating in a assembly file, and it is prefered to not change the original file.
3. Create a new xacro file called ur5e_robotiq.xacro in the urdf folder.

# Create URDF assembly
Inside the new xacro file, ur5e_robotiq.xacro, copy the following code:
```bash
<?xml version="1.0"?>
<robot name="ur5e_robotiq85" xmlns:xacro="http://ros.org/wiki/xacro">

   <!-- import main macro -->
  <xacro:include filename="robotiq_arg2f_85_macro.xacro" />
  <xacro:include filename="/home/mridulsongara/ur5_ws/src/universal_robot/ur_description/urdf/inc/ur5e_macro.xacro" />

  <!-- parameters -->
  <xacro:arg name="transmission_hw_interface" default="hardware_interface/EffortJointInterface"/>
  <!--
    legal values:
      - hardware_interface/PositionJointInterface
      - hardware_interface/EffortJointInterface
  -->

  <!-- gripper -->
  <xacro:robotiq_arg2f_85_gazebo 
    prefix="gripper_"
    transmission_hw_interface="$(arg transmission_hw_interface)"/>

  <!-- robot -->
  <!-- Instantiate the ur5e robot -->
  <xacro:ur5e_robot prefix="ur5e_"/> 

  <!--
    Attach the Gazebo model to Gazebo's world frame.
  -->
  <link name="world"/>
  <joint name="world_joint" type="fixed">
    <parent link="world"/>
    <child link="ur5e_base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

 <!-- Add a fixed joint connecting the gripper to the robot wrist -->
 <joint name="gripper_joint" type="fixed">
  <parent link="ur5e_tool0"/>
  <child link="gripper_base_link"/>
</joint>
 
</robot>
```
<!--
1. Include the UR5e and Robotiq macro xacro files:
```bash
<robot name="ur5e_robotiq" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="ur5e_macro.xacro"/>

  <xacro:include filename="robotiq_arg2f_85_macro.xacro"/>

</robot>
```
2. Instantiate the UR5e robot macro with appropriate prefix:
```bash
<xacro:ur5e_robot prefix="ur5e_"/>
```
3. Instantiate the Robotiq gripper macro with appropriate prefix:
```bash
<xacro:robotiq_arg2f_85 prefix="gripper_"/>
```
4. Add a fixed joint connecting the gripper to the robot's wrist:
```bash
<joint name="gripper_joint" type="fixed">
  <parent link="ur5e_tool0"/>
  <child link="gripper_base_link"/>
</joint>
```
-->
- Run the xacro file to generate the URDF model:
```bash
rosrun xacro xacro ur5e_robotiq85.xacro > ur5e_robotiq85.urdf
```

# Create launch file for assembly
To visualize the combined UR5 and Robotiq gripper xacro model in RViz and control it with MoveIt:
<!--1. Launch RViz and set the 'Global Options' > 'Fixed Frame' to 'base'.
-->
<!--3. In the 'Displays' panel, set the 'RobotModel' display and choose the generated URDF file as the robot description.
-->
1. Launch MoveIt:

```bash
roslaunch moveit_setup_assistant setup_assistant.launch
```

2. Go through the MoveIt setup assistant and configure the UR5e robotiq robot model.
Following link is helpful while going through the setup assistant:

https://roboticscasual.com/ros-tutorial-how-to-create-a-moveit-config-for-the-ur5-and-a-gripper/


3. Generate the MoveIt configuration files from the setup assistant. While saving choose a new folder name (e.g. `assembly_moveit_config`) inside `ur5_ws/src` folder.

4. Rebuild the workspace and source the setup files:
```bash
cd ~/ur5_ws
catkin_make
source ~/ur5_ws/devel/setup.bash
```

# Launch assembly in RViz:

```bash
roslaunch assembly_moveit_config demo.launch
```




# References:
http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html
http://wiki.ros.org/Industrial/Tutorials/Create_a_MoveIt_Pkg_for_an_Industrial_Robot

Completed with the help of *https://claude.ai*

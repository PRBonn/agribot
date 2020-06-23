# AgriBot Project (Gazebo)

---

### About the *agribot_gazebo* package

this package contains the models of plants and the world which is designed to robot be spawned into it and it looks like a small farm. it has a launch file to spawn the robot and initialize the world. 

YouTube: [Agribot Simulation Demo](https://youtu.be/aYkh9gSbTDI)

**`NOTE`** make sure you have updated your gazebo and you have installed gazebo-ros and controller-manager packages before running the simulation. used these commands to install them properly.

```
sudo apt-get update
sudo apt-get install ros-<DISTRO_OF_YOUR_ROS>-gazebo-ros*
sudo apt-get install ros-<DISTRO_OF_YOUR_ROS>-controller-manager
```
the created simulation provides IMU, GPS, Camera and wheel Odometry on the same topics as actual robot which gives this opportunity to run same codes on the simulation as the actual robot without any changes.
The image below shows the final version of simulation area and a robot spawned into environment.


<div align="center"><img src="/doc/images/gazebo-crop.png" alt="gazebo-crop" width="400" title="gazebo-crop"/></div>

To launch the `agribot_gazebo` simply run: 
```
$ roslaunch agribot_gazebo agribot_sb_farm.launch
```

**`NOTE`** to find more information about gazebo environment and how to define robots in this simulator please visit [Gazebo website](http://gazebosim.org/tutorials/?tut=ros_urdf) 

the launch file is taking the configuration of an empty world which is a default layer in gazebo environment and then makes the farm model on top of it. also, with given coordinates spawns the robot too. 

**`NOTE`** the description of the robot is loading from `agribot_control` package which contains the controllers and main description of the robot in format of URDF and XACRO files. for more information visit [agribot_control](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_control.md)


```xml
<?xml version="1.0"?>
<launch>

  <!-- these are the arguments you can pass this launch file, for example paused:=true -->
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find agribot_gazebo)/worlds/farm.world"/>
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>

  <!-- Spawn a robot into Gazebo -->
  <param name="robot_description" command="$(find xacro)/xacro.py '$(find agribot_control)/urdf/agribot.xacro'"/>

  <node name="agribot_spawn" pkg="gazebo_ros" type="spawn_model" output="screen" respawn="false"
   args="-x 6 -y -1 -z 1.13 -Y 1.57079 -urdf -param robot_description -model agribot" />

  <!-- ros_control pioneer launch file -->
<!--   <include file="$(find agribot_control)/launch/agribot_control.launch" />
 -->
</launch>

```

--- 
* [The Main ReadMe](https://github.com/PRBonn/Agribot/blob/master/README.md)
* [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) 

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 []()









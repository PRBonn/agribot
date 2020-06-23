# AgriBot Project (Mocap)

<!-- <div align="center">
	<img src="/doc/images/joystick_top.png" alt="JoystickTop" width="450" title="JoystickTop"/>
</div> -->

---

## How to Run the *mocap_optitrack* package
the mocap_optitrack provides accurate indoor localization service, which recognizes position of the robot with respect to markers on the robot. To use it run its application on the PC, in the lab and open the project saved in the default directory with name *agribot* to set and apply all settings required to have the position of the robot published in the ROS platform.

for more information take a look at [mocap_optitrack](http://wiki.ros.org/mocap_optitrack)

*`NOTE`* please run the *roscore* command on the Raspberry Pi or remote PC, before running the *mocap_optitrack* application on the server PC in the lab.

To launch the *agribot_visualservoing* simply run: 
```
$ roslaunch mocap_optitrack mocap.launch
```

**`NOTE`** to have full functionality of navigation, you will need a localization service running to provide position of the robot. In case which you use the robot in the lab, use *mocap* system as main localization service. 

Node [/mocap_node]

Publications: 
 * /agribot/pose2d [geometry_msgs/Pose2D]
 * /amcl_pose [geometry_msgs/PoseStamped]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

Subscriptions: 
 * None

Services: 
 * None

--- 

**Parameters**
 * None

--- 

*`NOTE`* the topic */amcl_pose* is the topic which navigation uses to run algorithms and is defined in */mocap* frame.
And */mocap* frame has a static link to */map* with a relative shift to set the origins together. the link is defined in the */mocap.launch*.

--- 
* [The Main ReadMe](https://github.com/alirezaahmadi/Agribot/blob/master/README.md)
* [AgriBot Software](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api.md) 

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)










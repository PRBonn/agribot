# AgriBot Project (VisualServoing)

---

## About VisualServoing Method
The visualServoing is a technique which uses feedback information extracted from a vision sensor to control the motion of the robot. Its usage in case of AgriBot is to navigate the robot through the crop rows in the field. the main graph of this node is shown. 

<div align="center">
	<img src="/doc/images/VSg.png" alt="visualservoing" width="400" title="visualservoing"/>
</div>

In AgriBot navigation strategy, input image will receive from the camera mounted on the front of the robot and as the robot is supposed to work on field expected image is the scene of soil and some crop rows. Hence, features can efficiently be define based on crops and the background will always contain soil and some other plants out of the row, which based on experiments they should be weeds. In figure below the robot and its camera consolation are shown with respect to crop rows in the field.

<div align="center">
	<img src="/doc/images/crow.png" alt="robotinrow" width="400" title="robotinrow"/>
	<img src="/doc/images/croprow.png" alt="croprow" width="400" title="croprow"/>
	<img src="/doc/images/crg.gif" alt="vsout" width="700" title="vsout"/>
</div>

---

## About the *agribot_visualservoing* package
In *Agribot_visualservoing* package, a vision sensor as input is used to recognize crop rows, which in the launch file under */launch* directory in the package user change the input source, too. it has access to *agribot_local_planner* and navigation (including *map_server*) to set new goals in */map* frame.

To launch the *agribot_visualservoing* simply run: 
```
$ roslaunch agribot_visualservoing visualservoing.launch
```

**NOTE** to have full functionality, you will need a localization service running to provide position of the robot. In case which you use the robot in the lab, use *mocap* system as localization server.

for more information about *mocap* system refer to this page [Motion Capture](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/mocap.md) 

Node: [/agribot_vs]

Publications: 
 * /cmd_vel [geometry_msgs/Twist]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /amcl_pose [unknown type]
 * /odom [unknown type]
 * /usb_cam/image_raw [unknown type]
 * /zed/camera/left/image_raw [sensor_msgs/Image]

Services: 
 * None

--- 

**Parameters**
 * None

--- 
* [The Main ReadMe](https://github.com/alirezaahmadi/Agribot/blob/master/README.md)
* [AgriBot Software](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api.md) 

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)














# AgriBot Project (agribot_control)

---

## About the *agribot_control* package
This package contains the description of the robot in URDF and XACRO forms along with gazebo plug-ins to actuate the robot in the simulation environment. The initial URDF is extracted from SolidWorks software, in the mechanical design section. Also it contains all the required information about the TF and different coordinate systems used in the robot (all are defined same as the actual robot).

this package comes with some launch files which the most important one is the *agribot_rviz.launch* which loads the robot into parameter-server and then the RVIZ simulator can access to the robot description values and show the robot.

<div align="center">
	<img src="/doc/images/rvizagribot.png" alt="rvizagribot" width="400" title="rvizagribot"/>
</div>

To launch the agribot_control simply run: 
```
$ roslaunch agribot_control agribot_rviz.launch
```

The other launch file is a controller manager which runs with gazebo and tries to publish and control the status of all active joints loaded into gazebo in run-time. to run this controller first launch [agribot_gazebo](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_gazebo.md) and then used command below:

```
$ roslaunch agribot_control agribot_control.launch
```

--- 
* [The Main ReadMe](https://github.com/alirezaahmadi/Agribot/blob/master/README.md)
* [AgriBot Softwarea](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api.md) 

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)
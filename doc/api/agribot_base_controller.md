# AgriBot Project (Servo Controller)

<!-- <div align="center">
	<img src="/doc/images/joystick_top.png" alt="JoystickTop" width="450" title="JoystickTop"/>
</div> -->

---

## Table of Contents
- [Kinamtics of Agribot](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_base_controller.md#kinamtics-of-agribot)
  - [About the *agribot_base_controller* package](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_base_controller.md#about-the-agribot_base_controller-package)
  - [Driving front Wheels](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_base_controller.md#1driving-front-wheels)
  - [Kinematics model](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_base_controller.md#2kinematics-model)
  - [Providing Odometry data](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_base_controller.md#3providing-odometry-data)
  - [Handling The Errors and Alarms](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_base_controller.md#4-handling-the-errors-and-alarms)
  - [How to Launch *agribot_base_controller*](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_base_controller.md#how-to-launch-agribot_base_controller)

---

## Kinamtics of Agribot

In standard vehicles, front wheels are driven and the car is steered by changing their direction which it means the rear wheels are just pulled by torque generated from front wheels. In this configuarion, back wheels are just used to hold car balanced and increase the maneuverability. In AgriBot platform, rear wheels are used to steer the robots motion and front wheels are driven by two geared DC motors. Generally, vehicles with streering system at the back are used in cases which high maneuverability is necessary on low-speed mode, such as forklifts. The steerable wheels of the AgriBot are able to turn more than 90deg to the left and right. Such a robot is highly maneuverable at a low speed. To properly controll a four-wheel vehicle like AgriBot, one constraint needs to be hold to let inner and outer wheels turn slip- free.

<div align="center">
  <img src="/doc/images/MDrive.png" alt="KDrive" width="400" title="KDrive"/>
  <img src="/doc/images/cot.png" alt="constraint" width="200" title="constraint"/>
</div>
where, δi and δo are inner and outer steer angles, respectively.

## About the *agribot_base_controller* package

This package contains all the dependencies and implementations related to driving the front wheels and the implementation of the kinematic model. The main core is based on [md49_base_controller](http://wiki.ros.org/md49_base_controller) which some modifications and improvements are applied to customize it for Agribot platform.

### 1.Driving front Wheels
This package comes with the name of *md49_base_controller* and offers functions to connect to the MD49 embedded drivers. AgriBot platform is equipped  with two geared DC-motors and respectively two drivers which, are controlled by the Raspberry Pi (Local processor on AgriBot) via USB (Virtual Com-Port). Image below shows the driver and actuator model with related links.

<div align="center">
  <img src="/doc/images/emg49.png" alt="EMG49" width="200" title="EMG49"/>
  <img src="/doc/images/md49_1.png" alt="MD49" width="200" title="MD49"/>
</div>

* [Driving motors - EMG49](https://robot-electronics.co.uk/products/drive-systems/emg49-gearmotor-with-encoder.html)
* [front wheel's Driver - MD49](https://robot-electronics.co.uk/md49-24v-5a-dual-h-bridge-motor-driver.html)

The image below shows all connections related to front actuators. 

<div align="center">
  <img src="/doc/images/md49con.png" alt="md49con" width="600" title="md49con"/>
</div>

### 2.Kinematics model

As it's mentioned earlier it dose all the calculations related to the kinematics of the robot. This means, its always subscribing to the */cmd_vel* topic (topic which provides velocity fields), and converts them to proper data to feed the drivers. Here, as the motion is dependent of actuation of front and back wheels, package needs to publish on the topic */servo_poses* which is subscribed by *agribot_servo* to orient back wheels properly with respect to the desired velocities and used kinematics model.

The implementation of different kinematic models are done in a library called *agribt_kinematics.h* which already contains three different kinematic models (differential model, Ackermann model, improved Ackermann model) and is switchable by the user. 

*`NOTE`* the default kinematic model which performs the best, is the *Improved Ackermann*, and its recommended to be used. Also the differential model is not usable with new structures implemented for back wheels.

to change the kinematic model, change the value of *KinematicModel* as a variable defined in *agribt_kinematics.h*. this parameter is not available as *ros-param* variable because its not supposed to  change a lot.

the orientation of local coordinate system on the robot which odometry will use it is shown in figure below.

<div align="center">
  <img src="/doc/images/robot_c.png" alt="robot_coordinate" width="400" title="robot_coordinate"/>
</div>

### 3.Providing Odometry data

This package provide the odometry data from different sources (wheel odometry and mocap odometry) along with driving the robot. 

* The mocap odometry is available if the robot is running in the lab with mocap system running and publishing the topic */amcl_pose* as the 2D/3D position of the robot and the functions in the *agribt_kinematics.h* converts it to odometry data, also initializes the wheels odometry at the beginning if its available.
* wheels odometry always is available and is published on its specific topic too.

*`NOTE`* as long as the wheels odometry data is dependent of kinematics model, its recommended to use *Improved-Ackermann* model because its prividing more accurate odomatry data. images below shows the odomatery results of *Ackermann* and *Improved-Ackermann* models.
* Green track is the mocap odomtery, as ground truth, and in the left image the *Ackermann* models odometry is shown beside *Improved-Ackermann* models odometry on the right.

<div align="center">
  <img src="/doc/images/ackerodom.png" alt="ackerodom" width="300" title="ackerodom"/>
  <img src="/doc/images/ackermmixodom.png" alt="ackermmImproved" width="300" title="ackermmImproved"/>
</div>

for more information about *mocap* system refer to this page [Motion Capture](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/mocap.md) 

### 4. Handling The Errors and Alarms
As it has access to Drivers and their registers (internal variables/status like voltage, temperature, etc...) its able to detect errors and make some proper actions to solve the problem or warn them through alarms.  therefore, two relays are controlled by this node which are actually controlling the power of actuators. This means, in case of having any problem in connection it could restart the drivers and re-establish the connections to the  drivers, also it makes some alarms via a buzzer. form more information about the Relays and Buzzers and their electrical connections refer to [*AgriBot Electrical Design*](https://github.com/alirezaahmadi/Agribot/blob/master/doc/elec.md)


### How to Launch *agribot_base_controller*
To launch it simply run: 

```
$ roslaunch md49_base_controller md49_base_controller_local.launch
```
**`NOTE`** In starting time of the base, buzzer will make 2 short beeps (to indicate that base is launched). After hearing the beeps you should wait 5 ~ 10 seconds to be able to drive the robot (this time is necessary to establish the connection between embedded-ROS nodes).

Node [/md49_base_controller]

**Publications**: 
 * /ServoPose [std_msgs/Int16MultiArray]
 * /md49_data [md49_messages/md49_data]
 * /md49_diff [md49_messages/md49_diff]
 * /md49_encoders [md49_messages/md49_encoders]
 * /mocap_odom [nav_msgs/Odometry]
 * /odometry/raw [nav_msgs/Odometry]
 * /rosout [rosgraph_msgs/Log]
 * /tf [tf2_msgs/TFMessage]

**Subscriptions**: 
 * /amcl_pose [unknown type]
 * /cmd_vel [unknown type]
 * /md49_diff [md49_messages/md49_diff]
 * /md49_encoders [md49_messages/md49_encoders]

**Services**: 
 * /md49_base_controller/get_loggers
 * /md49_base_controller/set_logger_level


--- 

**Parameters**
 * /md49/acceleration: 4
 * /md49/mode: 0
 * /md49/regulator: True
 * /md49/speed_l: 128
 * /md49/speed_r: 128
 * /md49/timeout: True
 * /serialport/bps: 38400
 * /serialport/name_l: /dev/md49_l
 * /serialport/name_r: /dev/md49_r

---

*`NOTE`* this package can't be built on any remote PC completely, due to using some native libraries of Raspberry Pi like [*WiringPi*](http://wiringpi.com) to control GPIO pins.

--- 
* [The Main ReadMe](https://github.com/alirezaahmadi/Agribot/blob/master/README.md)
* [AgriBot Software](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api.md) 

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)














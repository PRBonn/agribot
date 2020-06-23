# AgriBot Project (Joystick)

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

---

## Table of Contents
- [How to Use JoyStick]()
  - [About the *agribot_joystick* package](https://github.com/alirezaahmadi/Agribot/blob/master/doc/joystick.md#about-the-agribot_joy-package)
  - [Buttons/Axis functionalities](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api/agribot_joystick.md#buttonsaxis-functionalities)

---

**`WARNING`** 
- always make sure your distance to the robot is less than 20 meters.
- In case of noticing any delay in responds from robot, restart the nodes.

---

### About the *agribot_joystick* package
the main package to establish connection between Joystick and Raspberry Pi on the robot is *joy_teleop* [github](https://github.com/ros-drivers/joystick_drivers/tree/master/joy) which is provided by ROS. The package *agribot_joystick* in up to generate specific velocity commands. 


To launch the joystick simply run: 
```
$ roslaunch agribot_joystick agribot_joystick.launch
```

**NOTE** its better to check battery of joystick before launching the node. in case of joystick being not connected/recognized node will throw some error on the screen.

Node name : /agribot_joystick

Publications: 
 * /ServoPose [std_msgs/Int16MultiArray]
 * /cmd_vel [geometry_msgs/Twist]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /joy [sensor_msgs/Joy]

Services: 
 * /agribot_joystick/get_loggers
 * /agribot_joystick/set_logger_level

---

### Buttons/Axis functionalities

button and axis on the joystick can control the linear and angular velocities of the robot in two different modes (indoor and outdoor) 
 - indoor mode has provides less power to motors than outdoor which uses maximum power (these parameters can be modified through config/.yaml files on the package).

functionality of each input is show in image below:

<div align="center">
	<img src="/doc/images/joystick_top.png" alt="JoystickTop" width="450" title="JoystickTop"/>
	<img src="/doc/images/joystick_front.png" alt="JoystickFront" width="450" title="JoystickTop"/>
</div>


As simple instructions to drive the robot:

1. hold Indoor-Deadman button and try to drive the robot forward and backward by give some throttle (on left throttle axis - marked as linear Velocity).
2. while holding Indoor-Deadman button and linear velocity axis pressed, To drive the robot to the left and right, push the Steering-axis (marked as Steering angle) and hold it while robot is moving. 

**NOTE** if you releasing the Steering axis, wheels will go back to central position, similarly for linear velocity axis, robot will move as long as the throttle has some angle. On top of all functionalities Deadman button should always be pressed too.

--- 

**Parameters**

* Tur_lin_scale (double, default: 2)
The amount to scale the joystick input for the command linear velocity output in turbo(outdoor) mode.
* Tur_ang_scale (double, default: 1)
The amount to scale the joystick input for the command angular velocity output in turbo(outdoor) mode.

* lin_scale (double, default: 0.25)
The amount to scale the joystick input for the command linear velocity output.
* ang_scale (double, default: 1.5)
The amount to scale the joystick input for the command angular velocity output.
* srv_scale (double, default: 90)
The amount to scale the joystick input for the command velocity output.

* axis_deadman (int, default: 5)  # L2 shoulder button
The joystick index to disables/enables the output cmd_vel message(indoor).
* axis_turbo (int, default: 4)   # L1 shoulder button
The joystick index to disables/enables the output cmd_vel message(outdoor).
* axis_servo (int, default: 6)   
The joystick index to disables/enables the output servo (steering wheels joints) message.
* axis_stop (int, default: 7)   
The joystick index to disable the output cmd_vel and reset position of servos.

* axis_linear (int, default: 1)
The joystick button index to control linear speed
* axis_angular (int, default: 2)
The joystick button index to control angular speed
* axis_servo (int, default: 0)
The joystick button index to control position of steering system (servos)

--- 
* [The Main ReadMe](https://github.com/alirezaahmadi/Agribot/blob/master/README.md)
* [AgriBot Software](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api.md) 

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)














# AgriBot Project (Servo Controller)

---

## Table of Contents
- How to Use *agribot_srvo*
  - [About AgriBot's Rear Actators](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md#about-agribots-rear-actators)
  - [Electrical Connection to System](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md#electrical-connection-to-system)
  - [Serial to USB Converter](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md#serial-to-usb-converter)
  - [About the agribot_servo package (Embedded-ROS)](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md#about-the-agribot_servo-package-embedded-ros)
  - [How to launch agribot_servo](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md#how-to-launch-agribot_servo)
  - [How to Access/Modify Arduino code](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md#how-to-accessmodify-arduino-code)
  - [References](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md#references)

---

### About AgriBot's Rear Actators 

AgriBot is equipped with two servos at the rear wheels which is servo controls the orientation of one of the wheels based on commands received from the motion controller node.  The motion controller is running under base_controller package. 

<div align="center">
	<img src="/doc/images/servomec1.png" alt="servocon" width="300" title="servocon"/>
	<img src="/doc/images/gear.png" alt="gear" width="300" title="gear"/>
</div>

### Electrical Connection to System

In AgriBot, there are tow units which are operated based on *Embedded-ROS* which officially its called (Ros-Serial)[http://wiki.ros.org/rosserial ]. Servo controller module uses a *Arduino-Uno* board to controll the actuators via stepper drivers. The connection used in this section is shown in image below:

<div align="center"><img src="/doc/images/servocont.png" alt="servocon" width="700" title="servocon"/></div>

* As shown in the image the some I/O pins are used to connect the stepper driver to the Arduino board which uses I/O pins listed in table below along with their functionality .

| Pins in Arduino  | Functionality | Connected to | 
| --- | --- | --- |
| 13  | LED | I/O  |
| A0  | Pin Num.2  | ADC - Left Potentiometer  |
| A1  | Pin Num.2  | ADC - right Potentiometer  |
| 5  | PUL | I/O  |
| 10  | DIR  | I/O |
| 6  | PUL  | I/O |
| 11  | DIR  | I/O  |

* Pin-out of *Arduino-Uno* and Schematic of Stepper-Driver  can be found here:

<div align="center">
	<img src="/doc/images/unopinout.png" alt="Uno pinout" width="500" title="Uno pinout"/>
	<img src="/doc/images/stepcon.png" alt="stepcon" width="800" title="stepcon"/>
</div>


#### Serial to USB Converter
USB/Serial Converter Turns a USB connection to the 5V TX and RX Arduino requires to communicate. This board converts a USB connection to the 5V TX and RX an Arduino requires for communication. You can connect straight to the Arduino Mini or other micro-controllers, allowing them to talk to the computer.

<div align="center">
	<img src="/doc/images/ft232rl.png" alt="ft232rl" width="300" title="ft232rl"/>
</div>

---

### About the *agribot_servo* package (Embedded-ROS)

**`NOTE`**
In case which you want to access the IMU data via using AgriBot local processor, you can skip next part and jump to [How to launch *agribot_imu*](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md#how-to-launch-agribot_imu).

In case you want to use the IMU module with another PC, please follow these steps:

1. Connect the module via Micro-USB  cable to PC.
2. Check the Com-Port name in:
```
$ cd /dev/ &
$ ls 
```
based on the name you find, using above command, change the Com-Port name in file 'servo.launch'.

```xml
<?xml version="1.0"?>
<launch>
  <node pkg="rosserial_python" type="serial_node.py" name="agribot_servo">
    <param name="port" value="/dev/servo"/>
  </node>
</launch>
```
Now you're ready to launch the node. please keep no the tutorials.

**`NOTE`**
if after launching the robot base controller you get any error like:

```xml
[ERROR] [1549010158.867290]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
```
don't pay attention to it, this is a unsolved package issue from the main developers, but the robot will work without any problem. its recommanded to check th topics to make sure of working fine, too.

---

## How to launch *agribot_imu*

To launch the *agribot_servo* simply run: 

```
$ roslaunch agribot_launch servo.launch
```

Node name : /agribot_imu

Publications: 
 * None

Subscriptions: 
 * servo_pose std_msgs/Int16MultiArray

Services: 
 * None

--- 

**Parameters**
 * None

--- 

### How to Access/Modify Arduino code

To modify the #agribot_servo* code, you should install [Arduino compiler](https://www.arduino.cc) or its plug-in on sublime [Arduin On Sublime](https://packagecontrol.io/packages/Arduino-like%20IDEarduino) and useful tutorial are given in Arduino website as well.

You can access the #agribot_servo* code via link below:
* [*agribot_servo* code](https://github.com/PRBonn/Agribot/tree/master/code/agribot_servo)

---
* [The Main ReadMe](https://github.com/PRBonn/Agribot/blob/master/README.md)
* [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) 

---
### References
* [Stepper motor](https://www.omc-stepperonline.com/closed-loop-stepper-motor/Nema-17-Closed-loop-Geared-Stepper-L60mm-Gear-Raio-511-Encoder-1000CPR.html)
* [Closed loop stepper driver](https://www.omc-stepperonline.com/closed-loop-stepper-driver/closed-loop-stepper-driver-0-70a-24-48vdc-for-nema-17-23-24-stepper-motor-cl57t.html)


--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 []()
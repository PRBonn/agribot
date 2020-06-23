# AgriBot Project (IMU)

---

## Table of Contents
- How to Use *agribot_imu*
  - [About the IMU Sensor](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md#about-the-imu-sensor-bosch-bno-055-mems-imu-sensor)
  - [About the agribot_imu package](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md#about-the-agribot_imu-package-embedded-ros)
  - [How to Launch *agribot_imu*](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md#how-to-launch-agribot_imu)
  - [How to Access/Modify Arduino code](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md#how-to-accessmodify-arduino-code)
  - [References](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md#references)

---

### About the IMU Sensor (Bosch - BNO-055 MEMS IMU Sensor)
If you've ever ordered and wire up a 9-DOF sensor, chances are you've also realized the challenge of turning the sensor data from an accelerometer, gyroscope and magnetometer into actual "3D space orientation"! Orientation is a hard problem to solve. The sensor fusion algorithms (the secret sauce that blends accelerometer, magnetometer and gyroscope data into stable three-axis orientation output) can be mind-numbingly difficult to get right and implement on low cost real time systems.

Bosch is the first company to get this right by taking a MEMS accelerometer, magnetometer and gyroscope and putting them on a single die with a high speed ARM Cortex-M0 based processor to digest all the sensor data, abstract the sensor fusion and real time requirements away, and spit out data you can use in quaternions, Euler angles or vectors.

Rather than spending weeks or months fiddling with algorithms of varying accuracy and complexity, you can have meaningful sensor data in minutes thanks to the BNO055 - a smart 9-DOF sensor that does the sensor fusion all on its own!  You can read the data right over I2C and Bob's yer uncle.

![alt text](https://cdn-shop.adafruit.com/product-videos/1024x768/2472-04.mp4 "IMU Demo (from Adafruit)")

The BNO055 can output the following sensor data:

* Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere
* Absolute Orientation (Quaterion, 100Hz) Four point quaternion output for more accurate data manipulation
* Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s
* Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2
* Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro Tesla (uT)
* Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
* Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2
* Temperature (1Hz) Ambient temperature in degrees Celsius

<!-- <div align="center">
	<img src="/doc/images/logicbat_1.png" alt="logicbat_1" width="200" title="Logic power battery switch"/>
</div>  -->

### Dimensions and Electrical Specifications
* Dimensions: 20mm x 27mm x 4mm / 0.8" x 1.1" x 0.2"
* Header holes begin 4mm from the mounting holes
* Mounting Hole dimensions: 20mm x 12mm apart
* Uses I2C address 0x28 (default) or 0x29
* Weight: 3g
* Operating voltage: 3.3-5v
* Operating current: max 100mA

### Electrical Connection to System

In AgriBot, there are tow units which are operated based on *Embedded-ROS* which officially its called (Ros-Serial)[http://wiki.ros.org/rosserial ]. IMU module uses a *Arduino-ProMicro* board to access the raw values provided by the sensor. The connection used in this section is shown in image below:

<div align="center"><img src="/doc/images/uart_imu.png" alt="BMS" width="400" title="BMS"/></div>

* As shown in the image the i2c protocol is used to connect the sensor module to arduino board which uses (SDA, SCL,VCC,GND) pins.
* Pin-out of *Arduino-ProMicro* and axes of BNO-055 can be found here:

<div align="center">
	<img src="/doc/images/promicropinout.png" alt="pro-micro pinout" width="400" title="pro-micro pinout"/>
	<img src="/doc/images/BNaxes.png" alt="BN axes" width="400" title="BN axes"/>
</div>

**`NOTE`**
if after launching the robot base controller you get any error like:

```xml
[ERROR] [1549010158.867290]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
```
don't pay attention to it, this is a unsolved package issue from the main developers, but the robot will work without any problem. its recommanded to check th topics to make sure of working fine, too.

---

### About the *agribot_imu* package (Embedded-ROS)

**`NOTE`**
In case which you want to access the IMU data via using AgriBot local processor, you can skip next part and jump to [How to launch *agribot_imu*](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md#how-to-launch-agribot_imu).

In case you want to use the IMU module with another PC, please follow these steps:

1. Connect the module via Micro-USB  cable to PC.
2. Check the Com-Port name in:
```
$ cd /dev/ &
$ ls 
```
based on the name you find, using above command, change the Com-Port name in file 'imu.launch'.

```xml
<?xml version="1.0"?>
<launch>
	<node pkg="rosserial_python" type="serial_node.py" name="agribot_imu">
		<param name="port" value="/dev/imu"/>
		<param name="baud" value="57600"/>
	</node>	
</launch>
```
Now you're ready to launch the node. please keep no the tutorials.

---

## How to launch *agribot_imu*

To launch the *agribot_imu* simply run: 

```
$ roslaunch agribot_launch imu.launch
```
In case which everything working correctly, the terminal should print out relative information. please attention to image below:

<!-- <div align="center">
	<img src="/doc/images/logicbat_1.png" alt="logicbat_1" width="200" title="Logic power battery switch"/>
</div>  -->

Node name : /agribot_imu

Publications: 
 * /imu (sensor_msgs/imu)

Subscriptions: 
 * None

Services: 
 * None

--- 

**Parameters**
 * None

--- 

### How to Access/Modify Arduino code

To modify the #agribot_imu* code, you should install [Arduino compiler](https://www.arduino.cc) or its plug-in on sublime [Arduin On Sublime](https://packagecontrol.io/packages/Arduino-like%20IDEarduino) and useful tutorial are given in Arduino website as well.

You can access the #agribot_imu* code via link below:
* [AgriBot-IMU code](https://github.com/PRBonn/Agribot/tree/master/code/agribot_imu)

---
* [The Main ReadMe](https://github.com/PRBonn/Agribot/blob/master/README.md)
* [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) 

---
### References
- [Adafruit Website](https://www.adafruit.com/product/2472)

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)
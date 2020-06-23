# AgriBot - A New Agricultural Mobile Robot Platform

---

**`WARNING`** please attention to warnings:
- In case of having some pins disconnected or broken, please first read the tutorials to debug the problem and **'do not try your luck, otherwise, definitely you will burn some part!!!'**, or you can contact with [Alireza Ahmadi](https://github.com/PRBonn) as the designer of the robot.
- In case of noticing any delay in responds from the robot, please restart the nodes and connections.
- Always check the main connections (power, USB cables, Joystick) before turning the robot 'ON'.

---

## Table of Contents
- [Electrical Design](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#electrical-design)
  - [Power UP the AgriBot](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#power-up-the-agribot)
  	- [Logic level Battery](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#logic-level-battery)
  	- [High level Battery](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#high-level-battery)
    - [How to Charge the Agribot](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#how-to-charge-the-agribot)
  - [Power Management Part](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#power-management-part)
  - [Serial Communication, I/O Pins](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#serial-communication-io-pins)
  	- [IMU node Connection](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#imu-connection)
  	- [RearServo's Connections](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#rearservos-connections)
  	- [Front Actuators Connections](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#front-actuators)
  - [Reset Relays](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#reset-relays)
  - [Alarm](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md#alarms)

---

## Electrical Design

Electrical section of the AgriBot includes controlling layers, sensors, battery management system (BMS) and actuators. The control layers’ structure is shown in below which is comprised of two different major layers, Low-level controllers and User layer which are containing DCM (Device Communication Manager), Sensors and User layers. The main program executed on Low-level controller runs on a Raspberry Pi 3 embedded controller and directly controls all the sensors and actuators, also runs master of ROS to manage all nodes working in the network. This layer is responsible for time synchronization of whole connections and I/O’s used in the process, like reading the IMU (Inertial Measurement Unit) and controlling all four actuators for driving and steering the robot. 

<div align="center">
  <img src="/doc/images/agribotlayered.png" alt="agribotlayered" width="500" title="agribotlayered"/>
</div>

Also, it runs the main nodes which drive the robot based on defined kinematics and provides a basic fusion of the sensors for localization out of IMU, wheel odometry and GPS. This layer of control is accessible via [SSH](https://www.hostinger.com/tutorials/ssh-tutorial-how-does-ssh-work#gref). This type of connection makes user able to connect and stream out the status of all sensors and to implement his own high-level algorithms though establishing a client node to ROS master which mainly is running on Raspberry Pi.

<div align="center">
  <img src="/doc/images/Sensors.png" alt="Sensors" width="400" title="Sensors"/>
  <img src="/doc/images/Controlcon.png" alt="Controlcon" width="400" title="Controlcon"/>
</div> 

---

## Power UP the AgriBot
The designed robot uses two batteries to power up all the actuators and electrical equipments. 
To turn on the robot, simply change the states of two switches in front of the robot (in the box) to *NO*. The images in below show the switches which control the power.

<div align="center">
  <img src="/doc/images/powerswitch.png" alt="powerswitch" width="400" title="battery switches"/>
</div>

### Logic level Battery
It's used to power up all the equipments working with logic level of voltage (3.3v ~ 5v). including Raspberry PI, Arduino controllers, Ethernet switch, USB hub and etc.

<div align="center">
	<img src="/doc/images/logicbat_1.png" alt="logicbat_1" width="200" title="Logic power battery switch"/>
	<img src="/doc/images/logicbat.png" alt="logicbat" width="200" title="Logic power battery"/>
</div> 

### High level Battery
This battery is specially used to supply actuators and their drivers.

<div align="center">
	<img src="/doc/images/lead_acid.png" alt="lead_acid" width="200" title="lead acid battery"/>
	<img src="/doc/images/highlevelswitch.png" alt="highlevelswitch" width="100" title="High level switch"/>
</div>

### How to Charge the Agribot
As robot has two different batteries, it uses two different chargers with sockets shown in image below to charge.

<div align="center">
  <img src="/doc/images/charge.png" alt="powercharge" width="400" title="battery charger sockets"/>
  <img src="/doc/images/charger.png" alt="powercharge" width="400" title="battery charger"/>
</div>  

*`NOTE`* both chargers have a LED to show the status of charge. 
* *RED* is charging...
* *GREEN* is full...


### HOW to Turn-Off the robot

to shutdown the robot first run the command below, to shutdown the raspberry pi and then swtich of both power keys shown in [here](https://github.com/PRBonn/Agribot/raw/master/doc/images/powerswitch.png)

```
sudo poweroff
```

---

## Power Management Part
In such heavy and equipped robot, which needs to operate for a long time in uneven and/or muddy fields with lots of sensors and actuators which should be supplied properly, the power supply gets really important, in such a way that, if one of the controllers or sensors can’t get sufficient amount of power, it will cause unexpected neglects in operation which can directly affect how precise the tasks are achieved by the robot. Hence, the AgriBot uses two separate rechargeable 24v batteries to power up electrical tools. All the actuators are supplied through a 24v/5A battery which isolates the electrical connection of drive part from digital and sensor parts which use other 24v/10A battery to run. This separation avoids from effecting noses arisen from motors and drivers on main processor and logic parts. Also, each logic device gets its power from a separate regulator channel and this guarantees to have a stable working period for all electrical stuff. 

Power distribution task is done by using a regulator board which provides different voltages. Regulator board has two input sockets and ten outputs which each output unit provides four different levels of voltages: 0v (GNG), 5v, 12v, 24v. In image below the configuration of input and output pins are marked.

<div align="center"><img src="/doc/images/BMSPinout.png" alt="BMSPinout" width="600" title="BMSPinout"/></div>

as its show in image below: all  supplied devices through BMS board are getting their power from bike-battery(with aluminum body).

<div align="center"><img src="/doc/images/bms.png" alt="BMS" width="600" title="BMS"/></div>

hence, all actuators are driven by other battery to isolate the logic level equipments from power section (to avoid getting disturbed by actuators produced noise).

---

## Serial Communication, I/O Pins
Some equipments like IMU, Drivers and Reset relays are connected to reaspberry pi by I/O pins and serial to USB converters. connection map is shown in image below.

<div align="center"><img src="/doc/images/gpoio_rasp.png" alt="raspberrypigpio" width="400" title="raspberry"/></div>

### IMU Connection
IMU node as its completely explained in [agribot_imu](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md) section is an Embedded-ROS node which uses an *Arduino-ProMicro* board to read the sensor and publish the topic */imu* in network. its connection is shown in image below:

<div align="center"><img src="/doc/images/uart_imu.png" alt="BMS" width="400" title="BMS"/></div>

### RearServo's Connections
Rear caster wheels are motorized, by adding a perpendicular joint to their main axis, therefore their rotation around the Z-axis can be controlled by motion controller node. Also, this capability is implemented through an Embedded-ROS node running on a *Arduino-UNO* board. 

<div align="center">
  <img src="/doc/images/servo.png" alt="servo" width="200" title="servo"/>
  <img src="/doc/images/Sdriver.png" alt="Sdriver" width="200" title="Sdriver"/>
</div>

* [Stepper motor](https://www.omc-stepperonline.com/closed-loop-stepper-motor/Nema-17-Closed-loop-Geared-Stepper-L60mm-Gear-Raio-511-Encoder-1000CPR.html)
* [Closed loop stepper driver](https://www.omc-stepperonline.com/closed-loop-stepper-driver/closed-loop-stepper-driver-0-70a-24-48vdc-for-nema-17-23-24-stepper-motor-cl57t.html)

The image below shows all connections related to rear-servos. 

<div align="center">
  <img src="/doc/images/servocon.png" alt="servocon" width="600" title="servocon"/>
</div>

To find out more about the controller node please visit [agribot_servo](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md) package, which tries to explain all the details about connections and controller node.

--- 

### Front Actuators

AgriBot platform is equipped  with two geared DC-motors which, are connected to the Raspberry Pi (Local processor on AgriBot) via USB (Virtual Com-Port). Image below shows the driver and actuator model with related links.

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

To find out more about the controller node please visit [agribot_base_controller](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_base_controller.md) package, which tries to explain all the details about connections and controller node.

---

## Reset Relays
As its explained in [agribot_base_controller](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_base_controller.md) in case of occurring any problem in serial communication with actuators (front/back) two relays are utilized to automatically reset the power of drivers and re-establish the connections. this part which contains a relay board is shown in image below:

<div align="center"><img src="/doc/images/relay.png" alt="relay" width="600" title="relay"/></div>

## Alarms

Also, a buzzer is used to warn the low battery or any other electrical/software faults through some alarms:
the module is shown in below:

<div align="center"><img src="/doc/images/buzzer.png" alt="buzzer" width="100" title="buzzer"/></div>

The Alarm code is shown in table below:

| Type of Alarm  | Repeat-Number | Repeat-Duration(ms) | Error/Notification | How to fix | 
| --- | --- | --- | --- | --- |
| Broken | 2 | 300  | Base launched | --- |
| Continues | -- | -- | Low Battery | [Debug The Power Status](https://github.com/PRBonn/Agribot/blob/master/doc/debug.md#debug-the-power-status) |
| Broken | 5 | 70  | Serial Communication Error | [Debug The Electrical Connections](https://github.com/PRBonn/Agribot/blob/master/doc/debug.md#debug-the-electrical-connections)

Also, a simple log of the error will be printed on the terminal which base is launched.

**`NOTE`** all the connections used to control these part via raspberry pi is illustrated in [Raspberry Pi GPIOs](https://github.com/PRBonn/Agribot/raw/master/doc/images/gpoio_rasp.png)

---
[The Main ReadMe](https://github.com/PRBonn/Agribot/blob/master/README.md)

*Section* 
- [Mechanical Design](https://github.com/PRBonn/Agribot/blob/master/doc/mec.md)
- [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) 
- [Easy Bring up & Record Data](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md) 
- [Debug](https://github.com/PRBonn/Agribot/blob/master/doc/debug.md)

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)









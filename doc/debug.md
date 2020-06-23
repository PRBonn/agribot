# AgriBot - Agricultural Mobile Robot Platform

---

## Table of Contents
- [Debug The Electrical Connections](https://github.com/PRBonn/Agribot/blob/master/doc/dedug.md#debug-the-electrical-connections)
- [Debug The Network Connection](https://github.com/PRBonn/Agribot/blob/master/doc/dedug.md#debug-the-network-connection)
- [Debug The Mechanical Parts](https://github.com/PRBonn/Agribot/blob/master/doc/debug.md#debug-the-mechanical-parts)

---

### Debug The Electrical Connections

The potential problems in electrical connections could be the I/O pins which are connecting different devices to each other. for example the relays, drivers, and embedded controller on IMU and servo controller part. 
to debug them the best way is to first read the explanations provided in the related section and then based on the maps of I/O pins and find the problem.

* whenever its required to do some electrical debug, please make sure that the power is off.
* Also, by checking the data-sheets of different components you can find out more about the type of connection.

**`NOTE`** further information about electrical connections of each part (like servos, IMU,..) is given in the related section under the [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md).

---

### Debug The Network Connection

In case of having any issue with Network and making connection to the Agribot please first, read the [How to Connect to AgriBot](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#how-to-connect-to-agribot), and in case which you couldn't find or solve the problem, follow the instructions below:


1. make sure all setting are set according to the [How to Connect to AgriBot](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#how-to-connect-to-agribot)

* In case of having problem with Ethernet connection: 
2. check the cable of Ethernet connection from Raspberry pi to the switch mounted on the robot.
3. check the Ethernet cable which connects your PC to the switch.
4. reset the Ethernet switch on the robot. 
5. create a static IP manually for your PC in range of IP: 192.168.0.x with Net-mask: 24
6. check your IP address by using 

```
$ ifconfig
```

7. On the Remote PC use command below to make sure raspberry pi in reachable in the network.

```
$ ping 192.168.0.100 
```
if IP was reachable then problem should be solved.

* In case of having problem with Wifi connection: 
2. reset the Wifi router.
3. create a static IP manually for your PC in range of IP: 131.220.233.x with Net-mask: 24
4. check your IP address by using 

```
$ ifconfig
```

5. On the Remote PC use command below to make sure raspberry pi in reachable in the network.

```
$ ping 131.220.233.190 
```
if IP was reachable then problem should be solved.

---

### Debug The Mechanical Parts

As all the mechanical parts are attached together by screws and nuts, its recommended to check the connections and tight them up sometimes, specially, in time of taking the robot to the fields or doing experiments with long run. 

The most important mechanical parts and connection which can affect the performances is listed here:

1. Main connection of both legs in front and the back of the robot. (please take care of the dimensions when you tight screws, use a meter to measure the distance between legs and match them to specification shown in [Mechanical Design](https://github.com/PRBonn/Agribot/blob/master/doc/mec.md)), also check the screws on the  profiles which are used to fix legs together.  

<div align="center"><img src="/doc/images/mec_con.png" alt="mec_con" width="500" title="mec_con"/></div>

2. Connection of servo motors at rear axes (where the shaft of caster wheel is connected to the shaft of servo motor using a coupler). check all the screws on the coupler part.

<div align="center"><img src="/doc/images/mec_servo.png" alt="mec_servo" width="300" title="mec_servo"/></div>

3. Connection of front wheels to the motors and the connection of motors to the U-shape hinge of front wheels.

<div align="center"><img src="/doc/images/mec_emg.png" alt="mec_emg" width="300" title="mec_emg"/></div>

---

[The Main ReadMe](https://github.com/PRBonn/Agribot/blob/master/README.md)

*Section*:
- [Mechanical Design](https://github.com/PRBonn/Agribot/blob/master/doc/mec.md)
- [Electrical  Design](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md)
- [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) 
- [Easy Bring up & Record Data](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md) 

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)








# AgriBot Project (launch)

---

### About the *agribot_launch* package
This package is a collection of all *.launch* files from all packages written for AgrBot robot. 


To launch one desired launch file from this package simply run:
```
$ roslaunch agribot_launch YOUR_DESIRED_LAUNCH_FILE.launch
```
here is a list of all important launch files stored in */launch* folder of this package:

1. **base.launch** is the main launch file to run the necessary packages to drive the robot. it is a collection of drive.launch, sensors.launch and etc..
2. **gps.launch** launches the gps node to connect to the U-blox module which is a PPP GPS antenna and receiver.
3. **zed.launch** runs the Zed camera node, and streams all types of images and topics into ROS platform.
4. **camera.launch** uses *uvc_camera* pckage to get the images from an input camera/webcam from specific port and with special resolution which are defined in the launch file.
5. **imu.launch** creates a Embedded-ROS node using a python package called *rosserial_python* and opens the */dev/imu* port to access the imu data. more information in [*agribot_imu*](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_imu.md)
6. **xsens_driver.launch** drives the xsens IMU sensor which should be connected to one of the Raspberry Pi USB ports. 
7. **servo.launch** creates another Embedded-ROS node to access the Stepper-Drivers to control rear caster wheels. for more information take a look at [*agribot_servo*](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_servo.md)
8. **px4flow.launch** launches a node to access the odometry data provided by *px4flow* sensor which runs an embedded-optical flow.
9. **drive.launch** launches nodes required to make the connection to all actuators in the back and front of the robot. Also launches the joystick to user be able to run the robot independently.
10. **sensors.launch** a collection of commands to launch all the sensors which might be running for a specific test.

--- 
* [The Main ReadMe](https://github.com/PRBonn/Agribot/blob/master/README.md)
* [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) 

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)











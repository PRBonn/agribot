# AgriBot - A New Agricultural Mobile Robot Platform

---

## Table of Contents
- [How to Power up the AgriBot](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#how-to-power-up-the-agribot)
   - [How to Charge the Agribot](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#how-to-charge-the-agribot)
- [How to Connect to AgriBot](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#how-to-connect-to-agribot)
  - [PC Setup](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#pc-setup)
  	- [Wifi Connection](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#wifi-connection)
  	- [Ethernet Connection](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#ethernet-connection)
  - [Bring up](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#bring-up)
  	- [Log into AgriBot PC using HDMI display](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#log-into-agribot-pc-using-hdmi-display)
  	- [Log into AgriBot PC using SSH connection](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#log-into-agribot-pc-using-ssh-connection)
  	- [Using Screen Tool](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#using-screen-tool)
  	- [Run the roscore](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#run-the-roscore)
  	- [Run the Base Controller of AgriBot](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#run-the-base-controller-of-agribot)
- [How to Use JoyStick](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#how-to-use-joystick)
- [Easy Bring up for AgriBot](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#easy-bring-up-for-agribot)
- [Record Data](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#how-to-record-data)
- [How to Turn-Off the robot](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md#how-to-turn-off-the-robot)

---

## How to Power up the AgriBot
The AgriBot uses two batteries. One battery is used to power up all the
actuators (front and back wheel motor). The second battery is used to supply
power to all the electrical equipments including the on-board raspberry- pi
and other sensors. To turn on the robot, simply change the states of two
switches in front of the robot (in the box) to *ON*. The image below shows the
switches to turn on the power.

<div align="center">
	<img src="/doc/images/powerswitch.png" alt="powerswitch" width="400" title="battery switches"/>
</div> 

### How to Charge the AgriBot
As robot has two different batteries, it uses two different chargers with sockets shown in image below to charge.

<div align="center">
	<img src="/doc/images/charge.png" alt="powercharge" width="400" title="battery charger sockets"/>
  <img src="/doc/images/charger.png" alt="powercharge" width="400" title="battery charger"/>
</div> 

You can find more information about the electrical design and connections in
[Electrical
Design](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md)

---

## How to Connect to AgriBot

These instructions are to guide the user to properly make the connection
between a remote PC (your PC) and the AgriBot on-board PC. You can connect to
the AgriBot using either Wifi or Ethernet using ROS platform and
[SSH](https://www.hostinger.com/tutorials/ssh-tutorial-how-does-ssh-work#gref)
connection.

### Remote PC setup

Preparing the Remote PC (your laptop) to communicate with the AgriBot.

#### Install Ubuntu and ROS on your PC
- to install Ubuntu you can use following instructions on suggested website [How to install Ubuntu](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0)
- to install ROS use [How to install ROS](http://www.ros.org/install/) 

**`NOTE`**: These instruction was tested on Ubuntu 14.04 LTS with ROS Indigo and Ubuntu 16.04 LTS with ROS Kinetic.
**`WARNING`** chose correct version of ROS based on version of your Ubuntu.

---

### Network Connection

There are two different methods to access the AgriBot's on-board PC (Raspberry Pi), via (a) Wifi and (b) Ethernet connection. 

**`NOTE`** In case you want to run expensive algorithms like online mapping or
record the data from vision sensors, its recommended to use the Ethernet
connection as it is more reliable and provides more speed.

**`WARNING`** If you want to take the robot outside of the lab, you must connect to the Ethernet switch which is mounted on the robot. Currently, WiFi connection is not supported outside the lab. 

For both WiFi and Ethernet connections, we will set the network properties as shown in the image below:

<div align="center"><img src="/doc/images/wifi.png" alt="Network-connection" width="1000" title="Network-connection"/></div>

To find more information about multiple machine structure in ROS visit [MultipleMachines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines).

#### Wifi Connection

We first need to set the IP address of the remote PC as the ROS_HOSTNAME in
order to enable communication between AgriBot on-board PC (Raspberry Pi) and
the remote PC. Make sure that the remote PC and AgriBot PC should be connected
to the same WiFi network. 

Enter the command below in terminal window of the remote PC to find out the IP
address of the remote PC.

```
$ ifconfig
```
Enter this command to open the .bashrc file with nano editor in terminal.

```
$ nano ~/.bashrc
```

Press `alt+/` to jump to the end line of the file.
Add these two lines to modify the address of local host in the `ROS_MASTER_URI` and `ROS_HOSTNAME`. 

```
# set raspi as the master#
export ROS_MASTER_URI=http://131.220.233.190:11311
export ROS_HOSTNAME=(IP OF REMOTE PC for example: 131.220.233.191)
```

* save the changes and to exit press */Ctrl + x*.

Then, source the bashrc with command below.

```
$ source ~/.bashrc
```

**`NOTE`**
Special Cases:
1) If you want to run the remote PC as both `ROS Master` and the host, both IP addresses (`ROS_MASTER_URI` and `ROS_HOSTNAME`) should be set equal to your IP address (on the remote PC)

2) If you're going use Agribot PC (raspberry pi) as ROS master both IP addresses (`ROS_MASTER_URI` and `ROS_HOSTNAME`) should be set equal to Agribot's IP address.

please pay attention to this [Network config](https://github.com/PRBonn/Agribot/raw/master/doc/images/wifi.png)

#### Ethernet Connection

To the Ethernet connection, first connect the remote PC via the Ethernet cable to the switch mounted on the robot. 
Next, set the IP address of your PC manually in the range of 196.168.0.x (except 196.168.0.100 as its used for robot):

After connecting the Ethernet cable, please follow below instructions:

- Test if the connection is properly made: (you should find the IP address of your PC in streamed text)

```
$ ifconfig
```
make sure specified IP address is shown under title of Ethernet connection.

Enter the below command to open the .bashrc file with nano editor in terminal.

```
$ nano ~/.bashrc
```

Press `alt+/` to jump to the end line of the file.
Add these two lines to modify the address of local host in the `ROS_MASTER_URI` and `ROS_HOSTNAME`.
```
# set raspi as the master#
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_HOSTNAME=(IP OF REMOTE PC for example: 192.168.0.101)

```

Then, source the bashrc with below command.

```
$ source ~/.bashrc
```

**`NOTE`**
Special Cases:
1) If you want to run the remote PC as both `ROS Master` and the host, both IP addresses (`ROS_MASTER_URI` and `ROS_HOSTNAME`) should be set equal to your IP address (on the remote PC)

2) If your going use Agribot PC (raspberry pi) as ROS master both IP addresses (`ROS_MASTER_URI` and `ROS_HOSTNAME`) should be set equal to Agribot's IP address.

**`NOTE`**
In case of hving any problem with network connections check the [Debug The Network Connection](https://github.com/PRBonn/Agribot/blob/master/doc/dedug.md#debug-the-network-connection).

---

### Logging into AgriBot
There are two different ways to access the OS running on the AgriBot PC:

#### Log into AgriBot PC using HDMI display
Simply use the HDMI port on the raspberry pi to stream out the display data. 

**`NOTE`** The user-name and password of AgriBot's local process is:
* USER-Name: igg
* Password: 12345

**`NOTE`** In the default mode, the raspberry Pi is booting without displaying the
Desktop to make it faster, in case of needing the desktop application use
*raspi-config* command to bring up the configure windows and in boot option
menu, you can change the mode of boot to *desktop*. more information about
[*raspi_config*](https://www.raspberrypi.org/documentation/configuration/raspi-config.md)

#### Log into AgriBot PC using SSH connection
this tool gives you this option to log into a remote PC from the terminal.
to log into AgriBot local processor use command below:

**Ethernet**
```
ssh igg@192.168.0.100
```
**WiFi**
```
ssh igg@131.220.233.190
```

Following are the login details for the AgriBot PC:
* USER-Name: igg
* Password: 12345

You can find more information about [SSH connection](https://help.ubuntu.com/community/SSH/OpenSSH/ConnectingTo). 

#### Using Screen Tool (Recommended for longer runs)

Use the screen tool in case which you want to disconnect your system or
shutdown the SSH connection and still have all the nodes running on the
AgriBot local PC. Screen is a console application that allows you to use
multiple terminal sessions within one window. The program operates within a
shell session and acts as a container and manager for other terminal sessions,
similar to how a window manager manages windows.[Reference
Link](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-screen-on-an-ubuntu-cloud-server)

**How to install screen**

Use “apt-get” to install on Ubuntu:
```
sudo apt-get update
sudo apt-get install screen
```
Run the application on terminal:

```
screen
```
**`NOTE`** To use the screen tool to run the nodes on AgriBot, just run the
command screen before running SSH commands on your own PC.


#### Run the *roscore*

In case you need to debug or develop a code, its better to run the `roscore`
separately to have it always running. Otherwise is each time you want to run
your program (using launch files) you will need to shutdown all nodes like
simulators or dependent nodes which you had run before, because they won't
recognize new roscore.

But, if you just want to run the robot and record data or have test, you don't need to run `roscore` separately.

To Run the ROS_MASTER run this command on your PC or on the Raspberry Pi:
```
$ roscore
```
more information about *roscore* in [ROS Wiki](http://wiki.ros.org/roscore). 

#### Run the Base Controller of AgriBot

Now we will run all nodes required to record data and drive the robot.

<!-- to run this command in case which you have run the *roscore* earlier, open
another terminal tab with `<Shift+Ctrl+T>` and follow instruction in part
[SSH](https://github.com/PRBonn/Agribot/blob/master/doc/connect.md#log-into-agribot-pc-using-ssh-connection).
 -->

```
$ roslaunch agribot_launch base.launch
```

**`NOTE`** In starting time of the base, buzzer will make 2 short beeps (to indicate that base is launched). After hearing the beeps you should wait 5 ~ 10 seconds to be able to drive the robot (this time is necessary to establish the connection between embedded-ROS nodes).


**`NOTE`**
if after launching the robot base controller you get any error like:

```xml
[ERROR] [1549010158.867290]: Unable to sync with device; possible link problem or link software version mismatch such as hydro rosserial_python with groovy Arduino
```
don't pay attention to it, this is a unsolved package issue from the main developers, but the robot will work without any problem. its recommanded to check th topics to make sure of working fine, too.


If all the nodes launch without an error, robot is ready to be
moved/controlled by joystick or some other nodes publishing */cmd_vel* topic.

More information about the packages and launch files are available in [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) section. 

**`NOTE`**
In case you don't used *base.launch* from *agribot_launch* package, you should
launch each node separately for sensors and actuators. for more
information about the *launch* files refer to [*agribot_launch*](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_launch.md)

**`NOTE`**  
If connection between Raspberry Pi and motor drivers (from motors) couldn't be
established correctly (in any case of having problem in connection or power or
etc...) relays on the robot will begin to switch on and off and buzzer will
make sound.

---

## How to Use JoyStick

to drive the robot with joystick, you should first launch agribot_joystick.launch file with command below:

```xml
roslaunch agribot_joystick agribot_joystick.launch
```
**`WARNINIG`** while testing the motion of the robot keep your distance from the robot. its better to stand on side of the robot. also the front of the robot is where the Zed camera is mounted.

There are two modes to control the robot:
 - Indoor Mode  : Low power mode for indoor operations. (Activation with Indoor-Deadman button)
 - Outdoor Mode : High power mode for outdoor operations. (Activation with Outdoor-Deadman button)

Indoor mode provides less power to motors whereas outdoor which uses maximum
power (These parameters can be modified through *config.yaml* files on the
*agribot_joystick* package).

Functionality of different buttons is shown in the image below:

<div align="center">
	<img src="/doc/images/joystick_top.png" alt="JoystickTop" width="450" title="JoystickTop"/>
	<img src="/doc/images/joystick_front.png" alt="JoystickFront" width="450" title="JoystickTop"/>
</div>

As simple instructions to drive the robot:

1. hold Indoor-Deadman button and try to drive the robot forward and backward by give some throttle (on left throttle axis - marked as linear Velocity).
2. while holding Indoor-Deadman button and linear velocity axis pressed, To drive the robot to the left and right, push the Steering-axis (marked as Steering angle) and hold it while robot is moving. 

**`NOTE`** if you releasing the Steering axis, wheels will go back to central position, similarly for linear velocity axis, robot will move as long as the throttle has some angle. On top of all functionalities Deadman button should always be pressed too.

For more information about joystick refer to [*agribot_joystick*](https://github.com/PRBonn/Agribot/blob/master/doc/api/agribot_joystick.md)

---

### How to Record Data

You can record a bag file using the following command:

*Example_1:* To record all topics in home folder:
```
$ rosbag record -a
```
*Example_2:* To suppress a topic:
```
$ rosbag record -q /chatter
```
```
$ rosbag record -O <place for the address of  hosting location to save the bag file with name.bag > <place to list of your topics to record, in case of wanting all topics to be recorded use -a >
for instance:
$ rosbag record -O Agribot_sugarbeet.bag /chatter
or:
$ rosbag record -O Agribot_sugarbeet.bag -a
```

The recorded data can be saved either on AgriBot local processor (which only
contains 32GB of storage) or on the remote PC.
To ease the recording process you can use a launch file with necessary
commands located on agribot_launch package. More description in [AgriBot
Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md)
section.

More information about recording bag file [ROS bag file](http://wiki.ros.org/rosbag/Commandline)

**`NOTE`** In case of recording sensors with dense output like Z-Camera or
Lidar, its recommended to use remote PC as main host for bag file. Otherwise
AgriBot processor will run out of storage and crash.


**`WARNING`** while recording bag files and driving robot around for long
time, keep checking all the connections, the status of ROS network, charge of
battery, because sometimes due to network faults or failures in electrical
connections bag files won't be usable at the end. (these cases happen when
robot has a electrical problem or there is some defects in ROS network or
sensor connections to remote PC, so its better to check them first!!!)

---

### How to Turn-Off the robot

to shutdown the robot first run the command below, to shutdown the raspberry pi and then swtich of both power keys shown in [here](https://github.com/PRBonn/Agribot/raw/master/doc/images/powerswitch.png)

```
sudo poweroff
```
---

[The Main ReadMe](https://github.com/PRBonn/Agribot/blob/master/README.md)

*Section* 
- [Mechanical Design](https://github.com/PRBonn/Agribot/blob/master/doc/mec.md)
- [Electrical  Design](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md)
- [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) 
- [Debug](https://github.com/PRBonn/Agribot/blob/master/doc/debug.md)

--- 
 by: [Alireza Ahmadi](https://github.com/alirezaahmadi)                                     
 University of Bonn- Robotics & Geodetic Engineering

 Alireza.Ahmadi@uni-bonn.de                             
 










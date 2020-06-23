# AgriBot Project (Connect to AgirBot)

These instructions are up to guide the user to properly make the connection between a remote PC and AgriBot local processor. Connection could be established by Wifi or Ethernet protocol using ROS platform and [SSH](https://www.hostinger.com/tutorials/ssh-tutorial-how-does-ssh-work#gref) connection.

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

---

## **Necessary Steps to do**:
* [Power up the AgriBot](https://github.com/alirezaahmadi/Agribot/blob/master/doc/elec.md#power-up-the-agribot)

--- 

## Table of Contents
- [How to Connect to AgriBot]()
  - [PC Setup](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#pc-setup)
  	- [Wifi Connection](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#wifi-connection)
  	- [Ethernet Connection](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#ethernet-connection)
  - [Bring up](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#bring-up)
  	- [Log into AgriBot PC using HDMI display](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#log-into-agribot-pc-using-hdmi-display)
  	- [Log into AgriBot PC using SSH connection](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#log-into-agribot-pc-using-ssh-connection)
  	- [Using Screen Tool](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#using-screen-tool)
  	- [Run the roscore](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#run-the-roscore)
  	- [Run the Base Controller of AgriBot](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#run-the-base-controller-of-agribot)

---

### PC Setup

**`WARNING`**: The instructions in this part corresponds to the Remote PC (your desktop or laptop PC) which will control AgriBot.

**`NOTE`**: This instruction was tested on Ubuntu 14.04lts with ROS Indigo and Ubuntu 16.04lts with ROS Kinetic.

#### Install Ubuntu and ROS on your PC
- to install Ubuntu you can use following instructions on suggested website [How to install Ubuntu](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop#0)
- to install ROS use [How to install ROS](http://www.ros.org/install/) 

**`WARNING`** chose correct version of ROS based on version of your Ubuntu.

---

### Network Connection

there are two different methods to access the AgriBot's local processor, Wifi and Ethernet connection. 

**`NOTE`** In case you want to run expensive algorithms like online mapping or recording the data from vision sensors, its recommended to use Ethernet connection as its more reliable and provides more speed. Also, if you want to take the robot outside of the lab you need a local connection through Ethernet switch which is mounted on the robot. 

Here is instructions required to connect to the robot using either Wifi or Ethernet protocols.

for both Wifi and Ethernet connection pay attention to image below, to properly set network properties:

<div align="center"><img src="/doc/images/wifi.png" alt="Network-connection" width="1000" title="Network-connection"/></div>

#### Wifi Connection

In case of using  Wifi connection to connect to the robot follow instructions given in this section to connection properly to the robot.

ROS platform requires IP addresses in order to communicate between AgriBot Embedded PC (Raspberry Pi) and the remote PC. The remote PC and AgriBot PC should be connected to the same Wifi router.
Enter the below command on the terminal window of the remote PC to find out the IP address of the remote PC.

```
$ ifconfig
```
Text strings in the rectangle is the IP address of the Remote PC.

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

Enter the below command to open the .bashrc file with nano editor in terminal.

```
$ nano ~/.bashrc
```

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

Press `alt+/` to end line of the file.
Add these two marked lines to modify the address of local host in the `ROS_MASTER_URI` and `ROS_HOSTNAME` with the IP address acquired from the above terminal window. 

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

Then, source the bashrc with below command.

```
$ source ~/.bashrc
```

**`NOTE`**
if you want to run the `ROS Master` on your own system, both IP addresses (`ROS_MASTER_URI` and `ROS_HOSTNAME`) should be set equal to your IP address, and some changes should be done AgriBot system which are explained in [Changing Master PC]().



#### Ethernet Connection

In case that you want to connect to the AgriBot through Ethernet protocol, you need to make the connection through Ethernet switch mounted on the robot. 
- the IP address should be set manually in you system in range of 196.168.0.x like image below:

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

After establishing the connection, please follow below instructions:

- fist test the connection with entering:

```
$ ifconfig
```
make sure specified IP address is shown under title of Ethernet connection.

Enter the below command to open the .bashrc file with nano editor in terminal.

```
$ nano ~/.bashrc
```

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

Press `alt+/` to end line of the file.
Add these two marked lines to modify the address of local host in the `ROS_MASTER_URI` and `ROS_HOSTNAME` with the IP address acquired from the above terminal window. 

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

Then, source the bashrc with below command.

```
$ source ~/.bashrc
```

---

### Bring up

**`WARNING`** 
- This instruction is intended to be run on the remote PC. If you are following above instruction, please do run the `roscore` command on AgriBot PC or just launch desired launch file.
- Make sure that IP address on each device is set correctly.

##### Log into AgriBot PC using HDMI display

**NOTE**: The terminal application can be found with the Ubuntu search icon on the top left corner of the screen. Shortcut key for terminal is Ctrl-Alt-T.

##### Log into AgriBot PC using SSH connection

**NOTE**: The terminal application can be found with the Ubuntu search icon on the top left corner of the screen. Shortcut key for terminal is Ctrl-Alt-T.


##### Using Screen Tool

In case you want to disconnect your system or shutdown the SSH connection and have the nodes running on the AgriBot local processor use screen.

Screen is a console application that allows you to use multiple terminal sessions within one window. The program operates within a shell session and acts as a container and manager for other terminal sessions, similar to how a window manager manages windows.[reference](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-screen-on-an-ubuntu-cloud-server) 

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
**`NOTE`** to use the screen tool to run the nodes on AgriBot, just run the command screen before running SSH commands on your own PC. 

image below is showing required steps:

<!-- <p align="center"><img src="https://cdn.discordapp.com/attachments/205245036084985857/481213000540225550/full_example.gif" /></p> -->

###### Run the roscore

In case you need to debug or develop a code, its better to run the roscore separately to have it always running. Otherwise is each time you want to run your program (using launch files) you will need to shutdown all nodes like simulators  or dependent nodes which you had ran before, because they won't recognize new roscore.

```
$ roscore
```
**`NOTE`** if you just need to run the robot to record data or have a performance, its not necessary to run the roscore separately.
###### Run the Base Controller of AgriBot

following command will run all required nodes to record data and drive the robot. 

to run this command in case which you have ran roscore before, open another terminal tab with `<Shift+Ctrl+T>` and follow instruction in part [SSH](https://github.com/alirezaahmadi/Agribot/blob/master/doc/connect.md#log-into-agribot-pc-using-ssh-connection).

In the new tab enter command below:

```
$ roslaunch agribot_launch base.launch
```

If every this is going well and all sensors and actuators are connected and connection are flawless, the terminal will represent below messages.

**`NOTE`** More information about Nodes and launch files are given in [AgriBot Software](https://github.com/alirezaahmadi/Agribot/blob/master/doc/api.md) section. 

**`NOTE`** if connection between Raspberry Pi and motor drives (from motors) cant be established correctly (in any case of faults in connection or power or etc) relays on the robot will begin to switch on and off and buzzer will make alarm of a continuous sound.

---
* [The Main ReadMe](https://github.com/alirezaahmadi/Agribot/blob/master/README.md)

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering
 
 Alireza.Ahmadi@uni-bonn.de                             
 [](https://www.AlirezaAhmadi.xyz)

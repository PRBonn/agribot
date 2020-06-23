# AgriBot - A New Agricultural Mobile Robot Platform

---

## About The Project
The aim of the project is to develop a robot to perform basic localization and navigation tasks in agricultural applications. More precisely, the robot should embody a platform that is able to navigate autonomously in the fields and offers enough room for the assembly of task specific actuators like herbicide nozzles or additional sensors like cameras.
To achieve this goal, three main tasks have to be solved: the hardware design, the localization and the navigation. The first stage of development is the mechanical design of the robot. Requirements for the design are, that the platform is lightweight and easy to transport while being big enough to pass over crop rows and having space for the mentioned additional equipment. On top of the design of the layout of the platform, the sensors and actuators used for localization and navigation have to be decided on. The result of this first stage is a platform that can drive while being steered by a joystick controller.
The second step during the development is the localization of the robot. To be able to navigate autonomously it is essential that the robot pose relative to the field is known. Task of the localization is to use the given sensor data and give an estimation of this pose that is precise enough for the last step, the navigation.
During the navigation, the pose estimate of the localization in combination with a map of the field is taken as input and the necessary motion commands to follow a given trajectory through the field are returned. Consequently, at this point in time the joystick controller is not needed any more, as the motion commands are generated by the navigation module and the robot drives autonomously.

<!-- <div align="center">
	<img src="/doc/images/mechanic.png" alt="mechanic" width="600" title="mechanic"/>
</div> -->

---

*Note that* this repository is maintaining by [Alireza Ahamdi](https://github.com/PRBonn). 
In case of facing any problem don't hesitate to constant me.

---
[The Main ReadMe](https://github.com/PRBonn/Agribot/blob/master/README.md)

*Section* 
- [Mechanical Design](https://github.com/PRBonn/Agribot/blob/master/doc/mec.md)
- [Electrical  Design](https://github.com/PRBonn/Agribot/blob/master/doc/elec.md)
- [AgriBot Software](https://github.com/PRBonn/Agribot/blob/master/doc/api.md) 
- [Record Data](https://github.com/PRBonn/Agribot/blob/master/doc/recorddata.md) 
- [Debug](https://github.com/PRBonn/Agribot/blob/master/doc/debug.md)

--- 
  by: [Alireza Ahmadi](https://github.com/alirezaahmadi)                                     
 University of Bonn- Robotics & Geodetic Engineering

 Alireza.Ahmadi@uni-bonn.de          











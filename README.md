# CubeSystem
Name of the project: Testing Inertial Tilting System

This project is meant as a learning experiance, all the designs of 3d prints, all the code(with one exception *) and all the control is made by me and not optimal or even a good way of doing it. The point of this is to try to understand how and why certain things work or not. 
Along the way of the projects many things have changed and ive learnd more things and if i would do a version 2 i would change quite alot of descisions made early on in the project.

ESP32 have a WIFI module added to it that is used to log data and tuning PID values, this is just for easy access and everything from the WIFI can be accessd or changed in the code directly.

Tools used for making this projet:
FDM printer - material PLA
Lasercutter - material acrylic
Microcontroller - ESP32 lolin 
12v Nidec 24H brushless servomotor 
IMU MPU6050

Future plans for the project:
Implement LQR for control.
Look into more robust control, at the moment its quite sensitive to disturbance.
Improving on the WIFI utalization.
Adding code for combining the sensors for angle calculations.
Adding physical breaks for the flywheels which will go into to try the cube go fram laying flat on the side to "stand up on one edge after the other until a corner is reachd".




Code exception *, for the IMU angle values and filtering a lib have been used for accessing and using the DMP on the MPU6050.


![cube 1](https://user-images.githubusercontent.com/62113309/212912992-b5791865-fc0b-4514-88b0-0c5c7d0c55a3.jpg)
![Cube 2](https://user-images.githubusercontent.com/62113309/212912999-57e4fcf3-598e-45d6-bb4e-4e84d58c898e.jpg)

# About this Repo

A Hochschule Praktikum of TUM, implementing a pid controller on f1tenth vehicle to realize autonomous driving on track.

In this Repo basic manual controls (Keyboard and Joystick control) and autonomous driving algorithm (PID-Controller) are implemented in simulation environment 
and in real track. Through this lab a thorough understanding of ROS, Lidar, F110 Simulation environment and PID control algorithm can be achieved.
And simulation environment is developed by University of Pennsylvania. 

##  Requirements:

Unbuntu 18

ROS melodic

## How to use
1. enter workspace folder

2. build project 

   ```
   catkin_make
   ```
   

3. source setup

   ``` c++
   source devel/setup.sh
   ```
4. launch the project

    ```c++
    roslaunch pid_simulator car_simulator.launch
    ```


   

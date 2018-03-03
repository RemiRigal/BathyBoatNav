# BathyBoatNav
![Status](https://img.shields.io/badge/Status-In%20Development-red.svg)
![ROS](https://img.shields.io/badge/ROS-Kinetic--Kame-green.svg)

Hydro/Rob Project - Guerlédan 2017/2018 - Navigation part  
This project aims at providing a web-based monitoring and planning UI for robots.  
This part contains all the ROS packages used for the communication between the robot and the web-server and the ones used for the control.



## Installation
The packages use the version Kinetic-Kame of the [ROS](http://www.ros.org/) middleware.  
ROS can be installed via apt for [Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu) and [Debian](http://wiki.ros.org/kinetic/Installation/Debian) based distributions.  

Once ROS is installed and the repository cloned, open a terminal in the cloned directory and install the ROS packages:  
```shell
catkin_make install
```
You should now be ready to go:  
First start the WebApp to create a mission ([see how here](https://github.com/RemiRigal/BathyBoatWeb/blob/master/README.md))  
Then launch the mission interpreter file which parses the mission given by the WebApp:  
```shell
roslaunch /launch/mission_interpreter.launch
```
Next in another terminal launch the helios file for an evironment with all the nodes for the control, navigation and sounders:  
```shell
roslaunch /launch/helios.launch
```

The robot is now ready to start the mission, use the WebApp to send the start command.   


If you want to start a simulation instead of a real mission, after launching the mission interpreter launch the following file:  
```shell
roslaunch /launch/simu_helios.launch
```


## Configuration
Coming soon.


## ROS Nodes
There are several nodes which perform different tasks:  
- *mission_interpreter* which parses the JSON mission file created by the user in the WebApp and converts the gps positions in Lambert. It sends the next point the robot has to reach when called by the node *regul_helios*
- *gps_converter* which converts the gps positions from latitude/longitude to Lambert and inversely
- *tcp_server_send* which sends data from the robot to the WebApp (positions, motors thrust, batteries level ...)
- *tcp_server_recv* which receives the commands from the WebApp (start, stop ...)
- *regul_helios* which do the regulation of the robot based on a PID and sends the commands to the *pololu* node
- *pololu* which sends commands converted in PWM to the motors 
- *simu_helios* which simulates the evolution of the robot in time and space in function of the commands
- *fsm* which is in charge of the state machine of the robot

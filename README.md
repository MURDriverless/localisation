
<!-- PROJECT LOGO -->
<br />
<p align="center">

  <h3 align="center"> WIP Localisation </h3>
</p>



<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [TortoiseBot](#tortoisebot)
  * [Requirements](#requirements)
  * [Installation](#installation)
* [MURSIM](#mursim)
  * [Requirements](#requirements)
  * [Installation](#installation)
* [Folder Structures](#folder-structures)
* [Todo](#todo)


<!-- ABOUT THE PROJECT -->
## About The Project
Developed for the 2020 Melbourne University Racing (MUR) Motorsports Driverless Project as part of the Spatial and Perception System for the Vehicle State Estimation Subsystem. Multiple rospy files are within this repo for different purposes, including one that was developed using mobile phone sensors. The most recent is `tortoisebotlistener.py` which is detailed throughout this README.md file. This is designed to filter odometry data from the `tortoisebot` simulation by using Kalman Filters. 

<!-- TORTOISEBOT -->
## Tortoisebot

### Requirements

* tortoisebot
* hector_gazebo
* common_msgs
* ouster_example
* ROS melodic

### Installation
0. Install 'sudo apt install ros-melodic-desktop-full' and 'sudo apt install python-catkin-tools'
1. Clone the repo to an existing ROS Workspace (use catkin build)
```sh
git clone https://github.com/mfkeane/avse.git
```
2. Clone required repos to the same ROS Workspace
```sh
git clone https://github.com/mfkeane/common_msgs.git
git clone https://github.com/mfkeane/tortoisebot.git
git clone https://github.com/stevenlee090/ouster_example.git
git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git
```
3. Install required libraries, mainly `teleop_twist_keyboard` for controlling the robot movement.
```
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
3. Build the ROS Workspace with catkin build
4. Source the ROS Workspace
```sh
source devel/setup.bash
```
5. From the ROS Workspace, which will open Gazebo and RViz:
```sh
roslaunch tortoisebot tortoisebot.launch model:="$(find tortoisebot)/urdf/tortoisebot.urdf.xacrp"
```
6. From a seperate terminal:
```sh
rosrun localisation tortoisebotlocalisation.py
```
7. To control the robot, run this from another seperate terminal:
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
8. In RViz, add the `RobotModel` display and the two odometry messages, `/odom` and `/mur/Odom`. The first is the raw noisy readings from the GPS, and the second is the filtered data from the GPS and IMU. Change the colour of these to differientate them. Set fixed frame to `odom` and uncheck any covariances. You may wish to set to TopDownOrtho view and reduce the tolerances and keep variables for clarity.

Once finished, you should end up with something like this\
![demo](assets/demo.gif)

<!-- MURSIM -->
## MURSIM

### Requirements

* mur_sim
* See full list under the mur_sim repo

### Installation
WIP
0. Install 'sudo apt install ros-melodic-desktop-full' and 'sudo apt install python-catkin-tools'
1. Setup ROS Workspace as per mur_sim readme
2. Clone this repo to the MURSIM existing ROS Workspace (use catkin build)
```sh
git clone https://github.com/MURDriverless/localisation.git
```
3. Build the ROS Workspace with catkin build
4. Source the ROS Workspace
```sh
source devel/setup.bash
```
5. From the ROS Workspace, launch mur_sim
6. From a seperate terminal:
```sh
rosrun avse tortoisebotlistener.py
```
7. To control the robot, run this from another seperate terminal:
```sh
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
or use rqt
8. In RViz, add the `RobotModel` display and the two odometry messages, `/odom` and `/mur/Odom`. The first is the raw noisy readings from the GPS, and the second is the filtered data from the GPS and IMU. Change the colour of these to differientate them. Set fixed frame to `odom` and uncheck any covariances. You may wish to set to TopDownOrtho view and reduce the tolerances and keep variables for clarity.


## Folder Structures

```
├── localisation
│   ├── README.md
|   └── localisation
|       ├── CMakeLists.txt
│       ├── package.xml
│       ├── setup.py
|       └── src
|           ├── __init__.py
|           └── scripts
|               ├── mursimlocalisation.py
|               └── tortoisebotlocalisation.py
```

## Todo
- [x] Further tune the Kalman Filters
- [x] MURSIM compatibility

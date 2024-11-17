# AiRL-Hockey
Implementation of a robotic arm playing Air Hockey autonomously using Reinforcement Learning.

This project was developed as part of the Smart Robotics course at UniMore, utilizing `ROS Noetic`, `Gazebo` for simulation, and `MoveIt` for motion planning.

## Pre-requisites
This project has been built and tested on an `Ubuntu 20.04 Focal Fossa` virtual machine. It has not been verified on newer versions of Ubuntu or ROS.

To install the required dependencies, run the following commands:
```
wget -c https://raw.githubusercontent.com/qboticslabs/ros_install_noetic/master/ros_install_noetic.sh
chmod +x ./ros_install_noetic.sh
./ros_install_noetic.sh
sudo apt install python3 python3-pip
```

## Installation
Clone the repository, install Python dependencies, and set up the catkin environment:
```
git clone https://github.com/SR-Project/AH.git
cd ./AH
pip3 install -r requirements.txt
cd ./catkin_ws
catkin_make
```

## Usage
1. Open a terminal in the `catkin_ws` folder and start the Gazebo simulation:
```
source build/setup.bash
roslaunch air_hockey demo.launch
```
Press the `Play` button in Gazebo.

2. Open a second terminal in the `catkin_ws folder` to start the Reinforcement Learning script controlling the robot:
```
source build/setup.bash
rosrun air_hockey move_joint.py
```

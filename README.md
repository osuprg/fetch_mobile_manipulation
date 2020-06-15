# fetch_mobile_manipulation

This repository provides reusable modules and code for mobile manipulation. It was built as part of my masters research. Although it's tested with the Fetch, most functionalities are agnostic to the robot. 

Tested on Ubuntu 16.04, ROS Kinetic, Gazebo 7. Uses the fetch packages in https://github.com/fetchrobotics/fetch_ros. 

<p align="center">
  <img src="/mobile_manipulation/images/top_view_random_goal.gif" alt="Sublime's custom image"/>
</p>

## Install dependencies

```
sudo apt-get update
sudo apt-get install ros-kinetic-fetch-* (note - will install few additional ones than necessary)
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin (for Analytic Inverse Kinematics)

Please download gazebo models from https://bitbucket.org/osrf/gazebo_models/src/default/, and put them in 
~/.gazebo/models so your gazebo GUI can find them. 
```

## Build

Make a new catkin_workspace, or change to an existing workspace's src directory. 

```
mkdir -p ~/fetch_ws/src
cd ~/fetch_ws/src
git clone https://github.com/osuprg/fetch_mobile_manipulation/
cd ~/fetch_ws/
catkin_make
```
##  Run

```
source ~/fetch_ws/devel/setup.bash
cd ~/fetch_ws/mobile_manipulation/src/
./collect_data.sh 1
```

This does a single run of mobile manipulation. 

## Modules

Useful modules can be found in the /mobile_manipulation/src folder. Components are seperated out by utility.

## Assumptions 

On this user-facing branch, I have only added general components from my research code. The position of the object to be grasped is assumed to be known to avoid PCL dependencies. Localization is done using fake-odometry 
provided by a Gazebo Plugin, as the focus is more on mobile-manipulation and not localization. In the future, I might open source my perception code. 

## TODO

● ROS/Gazebo Docker Image to avoid dependancy conflicts

● New Worlds

● Perception Pipeline


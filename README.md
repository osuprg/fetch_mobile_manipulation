# DownstreamTasks
Navigation with optimization for downstream tasks

Tested on Ubuntu 16.04, ROS Kinetic, Gazebo 7, PCL 1.8. Uses the fetch packages in https://github.com/fetchrobotics/fetch_ros. 

## Install dependencies

```
sudo apt-get update
sudo apt-get install ros-kinetic-fetch-* (note - will install few additional ones than necessary)
sudo dpkg -i PCL-1.8.0-Linux.deb (Point Cloud Library install)
```

## Build

● Make a new catkin_workspace, or change to an existing workspace's src directory. 

```
cd ~/catkin_ws/src/
git clone https://github.com/sritee/DownstreamTasks
cd ~/catkin_ws
catkin_make
```
## Setup Config

```
cd catkin_ws/src/navr/config/
gedit experiment_params.yaml (or with your favorite editor)
```

Now, in the config file, edit the log_folder, config_folder and other paths with the correct user name paths.
Don't forget to end all the paths with a '/'.

Example the log_folder is changed as follows.

```
log_folder: /home/sritee/catkin_ws/src/navr/logs/testing_one ----> /home/(your username)/path to logs folder/
```

## Run

```
cd ~/catkin_ws/src/navr/src/
./collect_data 1
```

This does a single run of fetch manipulation. 



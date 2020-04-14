#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 19:21:59 2020

@author: sritee
"""

import roslaunch
import rospy
import os
import subprocess
import sys
import time
import ConfigParser

config = ConfigParser.ConfigParser()
config.read('../config/experiment_params.yaml')

gazebo_section_name = 'gazebo'

#TODO -- use config.getint etc
render = (config.get(gazebo_section_name, 'render') == 'True')
single_trial_time = int(config.get(gazebo_section_name, 'experiment_duration')) #time taken for single trial -- TODO, Make this automatic by sending signal from ros node
fetch_bringup_time = int(config.get(gazebo_section_name, 'sleep_time_after_fetchspawn'))
launch_gazebo_path = config.get(gazebo_section_name, 'gazebo_launch')
launch_node_path = config.get(gazebo_section_name, 'move_script')

port=str(11311)
port_gazebo=str(11312)


os.environ["ROS_MASTER_URI"] = "http://localhost:"+ port
os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+ port_gazebo
   
print("ROS_MASTER_URI=http://localhost:"+ port + "\n")
print("GAZEBO_MASTER_URI=http://localhost:"+ port_gazebo + "\n")

ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

os.system("killall -9 gzclient")
os.system("killall -9 gzserver")

rospy.init_node('trials')


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch_gazebo = roslaunch.parent.ROSLaunchParent(uuid, [launch_gazebo_path])
launch_gazebo.start()

if not render:
    os.system("killall -9 gzclient")

rospy.sleep(fetch_bringup_time) #for fetch bringup

launch_node = roslaunch.parent.ROSLaunchParent(uuid, [launch_node_path])
launch_node.start()

rospy.sleep(single_trial_time)

launch_gazebo.shutdown()
launch_node.shutdown()

for process in ['gzclient', 'gzserver']:
    while os.popen("ps -Af").read().count(process) > 0:
        
        os.system("killall -9 {}".format(process))
        time.sleep(1)
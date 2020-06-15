#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 19:21:59 2020

@author: sritee
"""

import os
import subprocess
import time

from std_srvs.srv import Empty
import ConfigParser
import roslaunch
import rospkg
import rospy

import utils

rospack = rospkg.RosPack()
# get the file path for this package
navr_path = rospack.get_path('fetch_mobile_manipulation') + '/mobile_manipulation'
config_path = navr_path + '/config/'

def kill_launch_service(request):
    
    launch_gazebo.shutdown()
    launch_node.shutdown()
    
    for process in ['gzclient', 'gzserver']:
        while os.popen("ps -Af").read().count(process) > 0:
            
            os.system("killall -9 {}".format(process))
            time.sleep(0.25)
    
    global experiment_running
    experiment_running = False

config = ConfigParser.ConfigParser()
config.read(config_path + 'experiment_params.yaml')

gazebo_section_name = 'gazebo'

#TODO -- use config.getint etc
render = (config.get(gazebo_section_name, 'render') == 'True')
single_trial_time = int(config.get(gazebo_section_name, 'experiment_duration')) #time taken for single trial -- TODO, Make this automatic by sending signal from ros node
fetch_bringup_time = int(config.get(gazebo_section_name, 'sleep_time_after_fetchspawn'))
launch_gazebo_path = navr_path + config.get(gazebo_section_name, 'gazebo_launch')
launch_node_path = navr_path + config.get(gazebo_section_name, 'move_script')
world_file = navr_path + config.get(gazebo_section_name, 'world_file')

port=str(11311)
port_gazebo=str(11312)

experiment_running = True

os.environ["ROS_MASTER_URI"] = "http://localhost:"+ port
os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+ port_gazebo
   
print("ROS_MASTER_URI=http://localhost:"+ port + "\n")
print("GAZEBO_MASTER_URI=http://localhost:"+ port_gazebo + "\n")

ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))

os.system("killall -9 gzclient")
os.system("killall -9 gzserver")

rospy.init_node('trials')

default_world = "worlds/empty.world"
utils.replace_text_in_file(launch_gazebo_path, default_world, world_file)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

kill_message_service = rospy.Service('kill_launch', Empty, kill_launch_service)
 
try:
    launch_gazebo = roslaunch.parent.ROSLaunchParent(uuid, [launch_gazebo_path])
    launch_gazebo.start()
except:
    print('Failed to launch!')
    pass
finally:
    utils.replace_text_in_file(launch_gazebo_path, world_file, default_world) #Rewrite the correct world name in launch file

if not render:
    os.system("killall -9 gzclient")

rospy.sleep(fetch_bringup_time) #for fetch bringup

launch_node = roslaunch.parent.ROSLaunchParent(uuid, [launch_node_path])
launch_node.start()

start_time = time.time()

while experiment_running and ((time.time() - start_time) < single_trial_time):
    time.sleep(1)




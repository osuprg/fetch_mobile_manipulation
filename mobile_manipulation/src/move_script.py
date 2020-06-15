#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This is the main file
"""

import numpy as np
import pdb

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
import ConfigParser
import rospkg
import rospy

from gazebo_utils.pose_utils import GazeboPoseMaster
from grasping.grasping import GraspingClient
from head_control.head_control_utils import PanAndTiltClient
from localization.localization import LocalizationClient
from navigation.navigate import NavigationClient
from rviz_utils.rviz_utils import RvizMarkerPublisher

def sample_valid_navigation_goal(publish_goal_marker = True):
    
    if sample_both_sides:
        
        x_val = np.random.uniform(low = x_low , high = x_high)
        
        if np.random.rand() > 0.5: #bottom side
            theta = 1.57
            y_val =  np.random.uniform(low = 1.54, high = 1.7)
        
        else: #top side
            
            theta = -1.57
            y_val =  np.random.uniform(low = 2.84, high = 3)
            
    else:
        
        theta = yaw
        
        x_val = np.random.uniform(low = x_low , high = x_high)
        y_val =  np.random.uniform(low = y_low, high = y_high)
    
    if publish_goal_marker:
        
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = x_val
        goal_pose.pose.position.y = y_val
        pose_publisher.publish(goal_pose)
   
    return [x_val, y_val, theta]


if __name__ == '__main__':
    
    rospack = rospkg.RosPack()
    # get the file path for navr
    navr_path = rospack.get_path('fetch_mobile_manipulation') + '/mobile_manipulation'
    config_file_folder = navr_path + '/config/'
    
    config = ConfigParser.ConfigParser()
    config.read(config_file_folder + 'experiment_params.yaml')
    
    experiment_section_name = 'experiment_section'
    sample_random_nav_goal = (config.get(experiment_section_name, 'sample_random_nav_goal') == 'True')
    
    #TODO -- use config.getfloat, config.getint etc
    can_offset_x = float(config.get(experiment_section_name, 'can_offset_x'))
    can_offset_y = float(config.get(experiment_section_name, 'can_offset_y'))
    
    sample_both_sides = (config.get(experiment_section_name, 'sample_both_sides') == 'True') #THIS will override y_low and high if true
    
    x_low = float(config.get(experiment_section_name, 'sample_xlow')) + can_offset_x
    x_high = float(config.get(experiment_section_name, 'sample_xhigh'))  + can_offset_x
    y_low = float(config.get(experiment_section_name, 'sample_ylow'))  + can_offset_y
    y_high = float(config.get(experiment_section_name, 'sample_yhigh'))  + can_offset_y
    yaw = float(config.get(experiment_section_name, 'yaw'))
    
    success_height = float(config.get(experiment_section_name, 'success_height'))
    default_nav_goal = [-0.45, 1.65, yaw]
    #default_nav_goal = [-0.45, 2.90, yaw]
    default_nav_goal[0] += can_offset_x
    default_nav_goal[1] += can_offset_y
    
    log_directory = config.get(experiment_section_name, 'log_folder')
    
    constrain_wrist_flex_joint = (config.get(experiment_section_name, 'constrained_wrist_flex_joint') == 'True')
    
    rospy.init_node("demo")
    
    amcl = LocalizationClient() #TODO - WE ARE ACTUALLY USING ODOMETRY AS AMCL, clean this up
    # Setup clients
    move_base = NavigationClient()
    head_action = PanAndTiltClient()
    grasping_client = GraspingClient(group = "arm")      
    gazebo_client = GazeboPoseMaster()   
    pose_publisher =  RvizMarkerPublisher()
    #Move the base to be in front of the table
    rospy.loginfo("Setting initial pose")   
    #amcl.set_pose()
    rospy.loginfo("Moving to table...")
    if sample_random_nav_goal:
        nav_goal = sample_valid_navigation_goal()
        print('RANDOM GOAL')
    else:
        nav_goal = default_nav_goal
        
    gazebo_client.set_pose_relative('coke_can', can_offset_x, can_offset_y, 0)
    
    move_base.clear_costmap()
    #
    obj_pose_before = gazebo_client.get_pose('coke_can')
    
    move_base.goto(nav_goal[0], nav_goal[1], nav_goal[2]) #unpack goal
    
    #time.sleep(5)
    head_action.look_at_surroundings(pan_range = 30, tilt_range = 45) #build octomap
    
    obj_pose = gazebo_client.get_pose()
    grasping_client.pick(obj_pose)
    #Time to request to kill ourselves!
    try: #To avoid error that service call didn't return we wrap in try catch
        rospy.wait_for_service('kill_launch')
        kill_us = rospy.ServiceProxy('kill_launch', Empty)
        kill_us.call()
    except:
        pass

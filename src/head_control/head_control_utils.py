#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# Head Control utils for the fetch

import numpy as np

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import PointHeadAction, PointHeadGoal
import actionlib
import rospy

class PanAndTiltClient:
    
    def __init__(self, pan_tilt_action_name = 'head_controller/follow_joint_trajectory',
                  pan_joint_name = 'head_pan_joint',  tilt_joint_name = 'head_tilt_joint'):
        
        self.PAN_TILT_ACTION_NAME = pan_tilt_action_name
        self.client = actionlib.SimpleActionClient(self.PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        
        self.PAN_JOINT = pan_joint_name
        self.TILT_JOINT = tilt_joint_name
        
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
    
    def tilt_pan_head(self, pan = 0, tilt = 0, duration = 2): #input is in degrees
        
        pan = pan * np.pi / 180
        tilt = tilt * np.pi / 180
        
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(duration)
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = [self.PAN_JOINT, self.TILT_JOINT]
        goal.trajectory.points.append(point)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
    def look_at_surroundings(self, pan_range = 45, tilt_range = 45): #looks at surroundings to build octomap
        
        self.tilt_pan_head(pan = -pan_range, tilt = tilt_range)
        self.tilt_pan_head(pan = pan_range, tilt = tilt_range)
        self.tilt_pan_head(pan = 0, tilt = 0)
    
class PointHeadClient():
    
    def __init__(self, point_head_action_name = "head_controller/point_head"):
        
        self.POINT_HEAD_ACTION_NAME = point_head_action_name
        self.client = actionlib.SimpleActionClient(self.POINT_HEAD_ACTION_NAME, PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        
    def look_at(self, x = 1, y = 0, z = 0, frame = 'base_link', duration= 1.0):
        
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()
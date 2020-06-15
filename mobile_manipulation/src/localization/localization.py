#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import rospy
import tf


class LocalizationClient():
    
    def __init__(self, initial_pose_topic_name = 'initialpose', odom_topic_name = '/odom_fake'):
        """
        

        Parameters
        ----------
        initial_pose_topic_name : string, optional
            DESCRIPTION. topic on which we set initial pose. The default is 'initialpose'.

        Returns
        -------
        None.

        """

        self.pose_setter = rospy.Publisher(initial_pose_topic_name, PoseWithCovarianceStamped, queue_size=10)
        self.odom_topic_name = odom_topic_name
        
    def set_pose(self, x = 0, y = 0 , yaw = 0, frame='map'):
        """
        

        Parameters
        ----------
        x : float
            x value of the goal
        y : float
            y value of the goal
        yaw : float
            yaw of the goal
        frame : string, optional
            DESCRIPTION. The default is "map". This is the frame
            in which the pose is set for amcl to initialize.

        Returns
        -------
        None.

        """
        
        init_pose = PoseWithCovarianceStamped()
        
        init_pose.pose.pose.position.x = x
        init_pose.pose.pose.position.y = y
        
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw) #rotation about z axis
        init_pose.pose.orientation = Quaternion(*quat)
        
        init_pose.pose.pose.orientation.w = 1
        
        self.pose_setter.publish(init_pose)
        
    def get_pose(self):
        """
        
        Returns
        -------
        pose_val : pose
            return a pose messaging containing amcl pose.

        """
        pose_stamped = rospy.wait_for_message(self.odom_topic_name, Odometry)
        pose_val = pose_stamped.pose.pose
         
        return pose_val

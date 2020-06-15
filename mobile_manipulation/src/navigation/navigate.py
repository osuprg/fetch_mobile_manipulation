#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
import actionlib
import rospy
import tf

class NavigationClient():

    
    def __init__(self, move_base_name = "move_base", clear_costmap_srv_name = 'move_base/clear_costmaps'):
        """
        
        
        Parameters
        ----------
        move_base_name : string, optional
            DESCRIPTION. The default is "move_base".
        clear_costmap_srv_name : string, optional
            DESCRIPTION. The default is 'move_base/clear_costmaps'.

        Returns
        -------
        None.

        """

        self.client = actionlib.SimpleActionClient(move_base_name, MoveBaseAction)
        self.clear_costmap_srv_name = clear_costmap_srv_name
        self.costmap_clearer = rospy.ServiceProxy(self.clear_costmap_srv_name, Empty)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, yaw, frame="map"):
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
            in which the goal is specified.

        Returns
        -------
        None.

        """
      
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw) #rotation about z axis
        move_goal.target_pose.pose.orientation = Quaternion(*quat)
        
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        self.client.send_goal(move_goal)
       
        self.client.wait_for_result()

    def clear_costmap(self):
        """
        Clears the costmap built by the navigation stack
        Returns
        -------
        None.

        """
       
        rospy.wait_for_service(self.clear_costmap_srv_name)
        self.costmap_clearer.call()
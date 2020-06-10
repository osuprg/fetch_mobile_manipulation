#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from visualization_msgs.msg import Marker
import rospy

class RvizMarkerPublisher:
    

    def __init__(self, topic_name = 'can_pose_marker'):
        """
        

        Parameters
        ----------
        topic_name : string, optional
            DESCRIPTION. Topic onto which we should publish marker. The default is 'can_pose_marker'.

        Returns
        -------
        None.

        """
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Marker, queue_size = 10)
        
    def publish(self, pose, frame = "map"):
        """
        

        Parameters
        ----------
        pose : pose message.
           pose of the marker to publish
        frame : string, optional
            Frame in which the pose is specified. The default is "map".

        Returns
        -------
        None.

        """
        
        marker = Marker()
        marker.header.frame_id = frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose = pose.pose
           
        self.publisher.publish(marker)
       
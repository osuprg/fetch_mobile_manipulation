#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import copy

from gazebo_msgs.srv import GetModelStateRequest, GetModelState
from gazebo_msgs.srv import SetModelStateRequest, SetModelState
import rospy


class GazeboPoseMaster:
    
    def __init__(self, get_pose_srv_name = 'gazebo/get_model_state', set_pose_srv_name = 'gazebo/set_model_state'):
        """
        

        Parameters
        ----------
        get_pose_srv_name : string, optional
            DESCRIPTION. The default is 'gazebo/get_model_state'.
        set_pose_srv_name : TYPE, optional
            DESCRIPTION. The default is 'gazebo/set_model_state'.

        Returns
        -------
        None.

        """
        
        self.get_pose_srv_name = get_pose_srv_name
        self.set_pose_srv_name = set_pose_srv_name
        self.pose_getter = rospy.ServiceProxy(self.get_pose_srv_name, GetModelState)
        self.pose_setter = rospy.ServiceProxy(self.set_pose_srv_name, SetModelState)

    def get_pose(self, object_name = 'coke_can'):
        """
        

        Parameters
        ----------
        object_name : float, optional
            DESCRIPTION. The default is 'coke_can'.

        Returns
        -------
        obj_pose : pose
            The pose of the object

        """
        
        obj_pose_req = GetModelStateRequest()

        obj_pose_req.model_name = object_name

        rospy.wait_for_service(self.get_pose_srv_name)
        obj_pose = self.pose_getter(obj_pose_req)
        
        return obj_pose
    
    def check_grasp_success(self, object_name = 'coke_can', height_threshold = 0.75):
        """
        

        Parameters
        ----------
        object_name : TYPE, optional
            DESCRIPTION. The default is 'coke_can'.
        height_threshold : TYPE, optional
            DESCRIPTION. The default is 0.75.

        Returns
        -------
        success : bool
            Whether the pose of the object_name is greater than height_threshold

        """
        
        obj_pose_req = GetModelStateRequest()

        obj_pose_req.model_name = object_name

        rospy.wait_for_service(self.get_pose_srv_name)
        obj_pose = self.pose_getter(obj_pose_req)
        
        height = obj_pose.pose.position.z
        
        success =  height > height_threshold
        
        return success
    
    def set_pose_relative(self, object_name = 'coke_can', relative_x = 0, relative_y = 0, relative_z = 0):
        """
        
        This set the object of a pose relative to its current pose. Only
        position supported currently, no orientation.
        Parameters
        ----------
        object_name : TYPE, optional
            DESCRIPTION. The default is 'coke_can'.
        relative_x : float, optional
            DESCRIPTION. The default is 0.
        relative_y : float, optional
            DESCRIPTION. The default is 0.
        relative_z : float, optional
            DESCRIPTION. The default is 0.

        Returns
        -------
        None.

        """
        
        obj_pose_req = SetModelStateRequest()

        obj_pose_req.model_state.model_name = object_name
    
        obj_cur_pose = self.get_pose()
        obj_next_pose = copy.deepcopy(obj_cur_pose.pose)
        obj_next_pose.position.x += relative_x
        obj_next_pose.position.y += relative_y
        obj_next_pose.position.z += relative_z
        rospy.wait_for_service(self.set_pose_srv_name)
        
        obj_pose_req.model_state.pose = obj_next_pose
        self.pose_setter(obj_pose_req)
        
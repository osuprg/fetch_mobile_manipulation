#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""


import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
#from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import GetModelStateRequest, GetModelState
from visualization_msgs.msg import Marker
import pdb
import moveit_commander
import tf
import sys
import time
import tf2_ros
import tf2_geometry_msgs

import numpy as np

import datetime
import pandas as pd
from logger import CustomLogger

logger = CustomLogger()

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        
        quat = tf.transformations.quaternion_from_euler(0, 0, theta) #rotation about z axis
        
        move_goal.target_pose.pose.orientation.x = quat[0]
        move_goal.target_pose.pose.orientation.y = quat[1]
        move_goal.target_pose.pose.orientation.z = quat[2]
        move_goal.target_pose.pose.orientation.w = quat[3]
        
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        #log send goal time
        #Log current pose
        #log target pose as well
        self.client.send_goal(move_goal)
        logger.update_log('Navigation Start')
        
        self.client.wait_for_result()
        logger.update_log('Navigation End')
        
        #log current pose
        #log current time

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        
        self.PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'
        self.PAN_JOINT = 'head_pan_joint'
        self.TILT_JOINT = 'head_tilt_joint'
        self.PAN_TILT_TIME = 1.5
        
        self.client = actionlib.SimpleActionClient(self.PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        
    def look_at(self, x = 1, y = 0, z = 0, frame = 'base_link', duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
    def tilt_pan_head(self, pan = 0, tilt = 30): #input is in degrees
        
        pan = pan * np.pi / 180
        tilt = tilt * np.pi / 180
        
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(self.PAN_TILT_TIME)
        goal = FollowJointTrajectoryGoal()

        goal.trajectory.joint_names = [self.PAN_JOINT, self.TILT_JOINT]
        goal.trajectory.points.append(point)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):

    def __init__(self, group="arm"):
        
#        self.scene = PlanningSceneInterface("base_link")
#        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
#        self.move_group = MoveGroupInterface("arm", "base_link")
#
#        find_topic = "basic_grasping_perception/find_objects"
#        rospy.loginfo("Waiting for %s..." % find_topic)
#        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
#        self.find_client.wait_for_server()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface("base_link")
        self.group = moveit_commander.MoveGroupCommander(group)
        self.group.set_goal_tolerance(0.001)
        
        #gripper params
        self.gripper_closed_pos = 0  # The position for a fully-closed gripper (meters).
        self.gripper_open_pos = 0.10  # The position for a fully-open gripper (meters).
        self.MIN_EFFORT = 35  # Min grasp force, in Newtons
        self.MAX_EFFORT = 100  # Max grasp force, in Newtons
        self.gripper_action_server_name = 'gripper_controller/gripper_action'
        self.gripper_client = actionlib.SimpleActionClient(self.gripper_action_server_name, GripperCommandAction)
        self.gripper_client.wait_for_server(rospy.Duration(10))

        
    def pick(self, obj_gazebo_pose):
         
         #assuming object pose is in gazebo frame, need to transform to base link frame
#         
#         box_pose = PoseStamped()
#         box_pose.header.frame_id = 'base_link'
#         box_pose.pose.orientation.w = 1.0
#         box_pose.pose.position.x = 0.5
#         box_pose.pose.position.y = 0
#         box_pose.pose.position.z = 0.5
#         
#         self.scene.attach_box('base_link', 'table_top', box_pose, size = (1,1,1))
         
         pose_publisher.publish(obj_gazebo_pose)
         grasp_pose = PoseStamped()
         grasp_pose.pose.position = obj_gazebo_pose.pose.position

         quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) #rotation about z axis
         grasp_pose.pose.orientation.x = quat[0]
         grasp_pose.pose.orientation.y = quat[1]
         grasp_pose.pose.orientation.z = quat[2]
         grasp_pose.pose.orientation.w = quat[3]
         grasp_pose.header.stamp = rospy.Time.now()
         grasp_pose.header.frame_id = 'map'
         grasp_pose.pose.position.z+=0.3
         
         transform = self.tf_buffer.lookup_transform('base_link',
                                       'map', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

         grasp_pose_in_base = tf2_geometry_msgs.do_transform_pose(grasp_pose, transform)
        
         self.group.set_pose_target(grasp_pose)
         #pdb.set_trace()
         #log target pose in base
         #log current pose in base
         #log current time
         logger.update_log('Planning Start')
         p1 = self.group.plan()
         logger.update_log('Planning End')
         #pdb.set_trace()
         #print(p1)
         #log time after plan completed
         #log execution time start
         logger.update_log('Execution Start')
         self.group.go()
         #log execution time end
         self.move_gripper_linearly(grasp_pose_in_base, avoid_collisions = False)
         logger.update_log('Execution End')
         #time.sleep(1)
         self.gripper_open()
         #time.sleep(1)
         #self.gripper_close()
         #pdb.set_trace()
         self.gripper_close()
         time.sleep(1)
         self.move_gripper_linearly(grasp_pose_in_base, reduce_height_by = -0.2) #lift up
         
         #group.get_end_effector_link()

    def gripper_open(self):
        """Opens the gripper.
        """
        goal = GripperCommandGoal()
        goal.command.position = self.gripper_open_pos
        goal.command.max_effort = self.MAX_EFFORT
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        

    def gripper_close(self, max_effort= None):
        """Closes the gripper.
        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        
        goal = GripperCommandGoal()
        goal.command.position = self.gripper_closed_pos
        goal.command.max_effort = self.MAX_EFFORT
        self.gripper_client.send_goal(goal)
        self.gripper_client.wait_for_result()
        
    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         )

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         )

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    
   

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
            
    def move_gripper_linearly(self, grasp_pose, reduce_height_by = 0.20, avoid_collisions = False, eef_step = 0.001): #computes cartesian path and goes down by depth m
        
        waypoints = []
        
        waypoints.append(grasp_pose.pose)
        target_pose = copy.deepcopy(grasp_pose)
        
        target_pose.pose.position.z -= reduce_height_by
        waypoints.append(target_pose.pose)
        #pdb.set_trace()
        trajectory, fraction = self.group.compute_cartesian_path(waypoints, 0.01, 0, avoid_collisions)
        #pdb.set_trace()
        self.group.execute(trajectory) #execute previously planned trajectory
        #pdb.set_trace()
        

class AmclPose:
    
    def __init__(self):
        
        self.pose_setter = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    
    def set_pose(self, x = 0, y = 0 , theta = 0, frame='map'):
        
        init_pose = PoseWithCovarianceStamped()
        
        init_pose.pose.pose.position.x = x
        init_pose.pose.pose.position.y = y
        
        #TODO - Euler to Quartnernion change, now ignoring theta
        
        init_pose.pose.pose.orientation.w = 1
        
        self.pose_setter.publish(init_pose)
        
class GazeboPoseMaster:
    
    def __init__(self, get_pose_srv_name = 'gazebo/get_model_state'):
        
        self.get_pose_srv_name = get_pose_srv_name
        self.pose_getter = rospy.ServiceProxy(self.get_pose_srv_name, GetModelState)

    def get_pose(self, object_name = 'coke_can'):
        
        obj_pose_req = GetModelStateRequest()

        obj_pose_req.model_name = object_name

        rospy.wait_for_service(self.get_pose_srv_name)
        obj_pose = self.pose_getter(obj_pose_req)
        
        #pdb.set_trace()
        return obj_pose
    
    def check_grasp_success(self, object_name = 'coke_can', height_threshold = 0.75): #if height is greater than 1, we lifted it
        
        obj_pose_req = GetModelStateRequest()

        obj_pose_req.model_name = object_name

        rospy.wait_for_service(self.get_pose_srv_name)
        obj_pose = self.pose_getter(obj_pose_req)
        
        height = obj_pose.pose.position.z
        
        success =  height > height_threshold
        
        return success
       
class RvizMarkerPublish:
    

    def __init__(self, topic_name = 'can_pose_marker'):
        self.topic_name = topic_name
        self.publisher = rospy.Publisher(self.topic_name, Marker, queue_size = 10)
        
    def publish(self, pose):
        
       marker = Marker()
       marker.header.frame_id = "/map"
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
      
       #pdb.set_trace()
       self.publisher.publish(marker)
       

if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")
    
    # Make sure sim time is working
    while not rospy.Time.now():
        pass
    
    amcl = AmclPose()
    # Setup clients
    move_base = MoveBaseClient()
#    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    grasping_client = GraspingClient()
#        
    gazebo_client = GazeboPoseMaster()
#    
    pose_publisher =  RvizMarkerPublish()

##
#     Move the base to be in front of the table
    rospy.loginfo("Setting initial pose")   
    amcl.set_pose()
    rospy.loginfo("Moving to table...")
    move_base.goto(-0.40, 1.66, 1.57)
##
###    # Point the head at the can we want to pick
    #head_action.look_at(0.6, 0, 0, "base_link")
#    rospy.sleep(3)
    head_action.tilt_pan_head(pan = 0, tilt = 50)
    
    obj_pose = gazebo_client.get_pose()
    time.sleep(1)
    grasping_client.pick(obj_pose)
#    pdb.set_trace()
    
    logger.update_log('Success', gazebo_client.check_grasp_success())
    print(logger)
    logger.save('./first_run.pkl')
        
    
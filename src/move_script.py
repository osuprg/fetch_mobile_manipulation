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

import tf2_ros
import tf2_geometry_msgs

import numpy as np

import time

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

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

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
        import tf
        self.listener = tf.TransformListener()
        
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface("base_link")
        self.group = moveit_commander.MoveGroupCommander(group)
        self.group.set_goal_tolerance(0.005)
        
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
         grasp_pose = PoseStamped()
         grasp_pose.pose.position = obj_gazebo_pose.pose.position
         grasp_pose.pose.position.z += 0.3
         #grasp_pose.pose.position.x = 0.4
         #grasp_pose.pose.position.y = 0.4
         
         quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) #rotation about z axis
         #quat = [0.051, 0.686, -0.037, 0.725]
         grasp_pose.pose.orientation.x = quat[0]
         grasp_pose.pose.orientation.y = quat[1]
         grasp_pose.pose.orientation.z = quat[2]
         grasp_pose.pose.orientation.w = quat[3]
         #grasp_pose.pose.position = obj_gazebo_pose.pose
         #grasp_pose.header.frame_id = 'base_link'
         grasp_pose.header.stamp = rospy.Time.now()
         grasp_pose.header.frame_id = 'map'
         pdb.set_trace()
         self.group.set_pose_target(grasp_pose)
         pdb.set_trace()
         #p1 = self.group.plan()
         #self.group.go()
         self.gripper_open()
         self.gripper_close()
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
        #return np.array([obj_pose.pose.position.x,obj_pose.pose.position.y, obj_pose.pose.position.z, 
                #         obj_pose.pose.orientation.x, obj_pose.pose.orientation.y, obj_pose.pose.orientation.z, obj_pose.pose.orientation.w ])
    
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
        
    gazebo_client = GazeboPoseMaster()
    
#
    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
#    rospy.loginfo("Setting initial pose")
    amcl.set_pose()
    rospy.loginfo("Moving to table...")
    move_base.goto(-0.46, 1.66, 1.57)

    # Point the head at the cube we want to pick
    head_action.look_at(1, 0, 0, "base_link")
    
    grasping_client.pick(gazebo_client.get_pose())
    
#
    # Get block to pick
#    while not rospy.is_shutdown():
#        rospy.loginfo("Picking object...")
#        grasping_client.updateScene()
#        cube, grasps = grasping_client.getGraspableCube()
#        #import pdb
#        pdb.set_trace()
#        if cube == None:
#            rospy.logwarn("Perception failed.")
#            continue
#
#        # Pick the block
#        if grasping_client.pick(cube, grasps):
#            break
#        rospy.logwarn("Grasping failed.")
#
#    # Tuck the arm
#    grasping_client.tuck()
#
#    # Lower torso
#    rospy.loginfo("Lowering torso...")
#    torso_action.move_to([0.0, ])
#
#    # Move to second table
#    rospy.loginfo("Moving to second table...")
#    move_base.goto(-3.53, 3.75, 1.57)
#    move_base.goto(-3.53, 4.15, 1.57)
#
#    # Raise the torso using just a controller
#    rospy.loginfo("Raising torso...")
#    torso_action.move_to([0.4, ])
#
#    # Place the block
#    while not rospy.is_shutdown():
#        rospy.loginfo("Placing object...")
#        pose = PoseStamped()
#        pose.pose = cube.primitive_poses[0]
#        pose.pose.position.z += 0.05
#        pose.header.frame_id = cube.header.frame_id
#        if grasping_client.place(cube, pose):
#            break
#        rospy.logwarn("Placing failed.")
#
#    # Tuck the arm, lower the torso
#    grasping_client.tuck()
#    torso_action.move_to([0.0, ])

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import os
import copy
import actionlib
import rospy
import copy
from math import sin, cos
#from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import GetModelStateRequest, GetModelState
from gazebo_msgs.srv import SetModelStateRequest, SetModelState
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
import ConfigParser

config_file_folder = '/home/sritee/catkin_ws/src/navr/config/'

logger = CustomLogger()
config = ConfigParser.ConfigParser()
config.read(config_file_folder + 'experiment_params.yaml')

experiment_section_name = 'experiment_section'
sample_random_nav_goal = (config.get(experiment_section_name, 'sample_random_nav_goal') == 'True')

#TODO -- use config.getfloat, config.getint etc
can_offset_x = float(config.get(experiment_section_name, 'can_offset_x'))
can_offset_y = float(config.get(experiment_section_name, 'can_offset_y'))

x_low = float(config.get(experiment_section_name, 'sample_xlow')) + can_offset_x
x_high = float(config.get(experiment_section_name, 'sample_xhigh'))  + can_offset_x
y_low = float(config.get(experiment_section_name, 'sample_ylow'))  + can_offset_y
y_high = float(config.get(experiment_section_name, 'sample_yhigh'))  + can_offset_y

default_nav_goal = [-0.40, 1.66, 1.57]
default_nav_goal[0] += can_offset_x
default_nav_goal[1] += can_offset_y

log_directory = config.get(experiment_section_name, 'log_folder')
#pdb.set_trace()
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
        logger.update_log('Base Planning Goal', np.array([x,y, theta])) #todo - resolve inconsistency
        logger.update_log('Base Planning Start')
        logger.update_log('Base Navigation Start Pose', amcl.get_pose())
        self.client.send_goal(move_goal)
        logger.update_log('Base Planning End')
        logger.update_log('Base Navigation Start')
        self.client.wait_for_result()
        logger.update_log('Base Navigation End')
        logger.update_log('Base Navigation End Pose', amcl.get_pose())
        
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

    def __init__(self, interface = 'pan_and_tilt'):
        
        self.PAN_TILT_ACTION_NAME = None
        self.POINT_HEAD_ACTION_NAME = None
        
        if interface == 'pan_and_tilt':
            self.PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory'
            self.client = actionlib.SimpleActionClient(self.PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        elif interface == 'point_head':
            self.POINT_HEAD_ACTION_NAME = "head_controller/point_head"
            self.client = actionlib.SimpleActionClient(self.POINT_HEAD_ACTION_NAME, PointHeadAction)
        else:
            raise ValueError('only pan_and_tilt and point_head supported')
            
        self.PAN_JOINT = 'head_pan_joint'
        self.TILT_JOINT = 'head_tilt_joint'
        self.PAN_TILT_TIME = 1.5
        
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        
    def look_at(self, x = 1, y = 0, z = 0, frame = 'base_link', duration=1.0):
        
        if self.POINT_HEAD_ACTION_NAME == None:
            raise Exception('using pan tilt interface, cannot point')
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
    def tilt_pan_head(self, pan = 0, tilt = 0): #input is in degrees
        
        if self.PAN_TILT_ACTION_NAME is None:
            raise Exception('using point head interface, cannot pan and tilt')
            
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
        
    def look_at_surroundings(self, pan_range = 45, tilt_range = 45): #looks at surroundings to build octomap
        
        self.tilt_pan_head(pan = 0, tilt = tilt_range)
        self.tilt_pan_head(pan = -tilt_range, tilt = tilt_range)
        self.tilt_pan_head(pan = pan_range, tilt = tilt_range)
        self.tilt_pan_head(pan = 0, tilt = 0)
        

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
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1)) #tf buffer length
        self.tf_buffer.clear()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface("base_link")
        self.group = moveit_commander.MoveGroupCommander(group)
        self.tolerance = rospy.get_param('move_group/arm/tolerance', default = 0.005)
        self.group.set_goal_tolerance(self.tolerance)
        
        #gripper params
        self.gripper_height_above = 0.2
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
         grasp_pose.pose.position.z+= self.gripper_height_above
         
#         base_to_map_transform = self.tf_buffer.lookup_transform('base_link',
#                                       'map', #source frame
#                                       rospy.Time(0), #get the tf at first available time
#                                       rospy.Duration(1.0)) #wait for 1 second
#
#         grasp_pose_in_base = tf2_geometry_msgs.do_transform_pose(grasp_pose, base_to_map_transform) #NOT USED
        
         self.group.set_pose_target(grasp_pose)
         #pdb.set_trace()
         #log target pose in base
         #log current pose in base
         #log current time
         logger.update_log('Arm Planning Start')
         
         plan = self.group.plan()
         logger.update_log('Arm Planning End')
         logger.update_log('Arm Planning Success', bool(plan.joint_trajectory.points))
         #pdb.set_trace()
         #print(p1)
         #log time after plan completed
         #log execution time start
         logger.update_log('Arm Execution Start')
         logger.update_log('Arm Execution Start Pose', amcl.get_pose())
         arm_execution_success = self.group.execute(plan)
         logger.update_log('Arm Execution Success', arm_execution_success)
         #log execution time end
         time.sleep(1)
         if not plan.joint_trajectory.points:
             return
         
         base_to_map_transform_updated = self.tf_buffer.lookup_transform('base_link',
                                       'map', #source frame
                                       rospy.Time(0), #get the tf at first available time
                                       rospy.Duration(1.0)) #wait for 1 second

         grasp_pose_in_base = tf2_geometry_msgs.do_transform_pose(grasp_pose, base_to_map_transform_updated)
         #pdb.set_trace()
         cartesian_servoing_success = self.move_gripper_linearly(grasp_pose_in_base, reduce_height_by = 0.0, avoid_collisions = True)
         time.sleep(0.25)
         logger.update_log('Cartesian Servoing Success', cartesian_servoing_success)
         #pdb.set_trace()
         cartesian_moving_down_success = self.move_gripper_linearly(grasp_pose_in_base, reduce_height_by = 0.1, avoid_collisions = False)
         logger.update_log('Cartesian Linear Success', cartesian_moving_down_success)
         logger.update_log('Arm Execution End')
         logger.update_log('Arm Execution End Pose', amcl.get_pose())
         time.sleep(1)
         #pdb.set_trace()
        
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
            
    def move_gripper_linearly(self, grasp_pose, reduce_height_by = 0.20, avoid_collisions = False, eef_step = 0.005): #computes cartesian path and goes down by depth m
        
        #self.group.set_goal_tolerance(0.0005)
        waypoints = []
        
        waypoints.append(grasp_pose.pose)
        target_pose = copy.deepcopy(grasp_pose)
        
        target_pose.pose.position.z -= reduce_height_by
        waypoints.append(target_pose.pose)
        #pdb.set_trace()
        trajectory, fraction = self.group.compute_cartesian_path(waypoints, eef_step, 0, avoid_collisions)
        #pdb.set_trace()
        cartesian_execute_success = self.group.execute(trajectory) #execute previously planned trajectory
        #self.group.set_goal_tolerance(0.001)
        #pdb.set_trace()
        return cartesian_execute_success
        

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
        
    def get_pose(self):
        
         pose_stamped = rospy.wait_for_message('/odom_fake', Odometry)
         #pdb.set_trace()
         pose_val = pose_stamped.pose.pose
         
         return pose_val
        
class GazeboPoseMaster:
    
    def __init__(self, get_pose_srv_name = 'gazebo/get_model_state', set_pose_srv_name = 'gazebo/set_model_state'):
        
        self.get_pose_srv_name = get_pose_srv_name
        self.set_pose_srv_name = set_pose_srv_name
        self.pose_getter = rospy.ServiceProxy(self.get_pose_srv_name, GetModelState)
        self.pose_setter = rospy.ServiceProxy(self.set_pose_srv_name, SetModelState)

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
    
    def set_pose_relative(self, object_name = 'coke_can', relative_x = 0, relative_y = 0, relative_z = 0):
        """
        For now, this sets the pose relative to the can default pose in gazebo, only position supported
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
       
def shutdown_process():
    #currently doing logging during node shutdown
    cnt = 1
    if not os.path.exists(log_directory):
        os.mkdir(log_directory)
    while os.path.exists(log_directory + 'run_' + str(cnt) + '.pkl'):
        cnt+=1    
    rospy.loginfo("Saving Log file")
    
    config_file_list = [config_file_folder + f for f in os.listdir(config_file_folder)]
    logger.save(logdir = log_directory, name = 'run_'  + str(cnt) + '.pkl', 
                config_files= config_file_list)
    
    
def sample_valid_navigation_goal(publish_goal_marker = True):
    
    #TODO -  currently this function has hardcoded sample bounds. Make it map specific
    #pose angle theta is also fixed.
    
    theta = 1.57 #currently dixed
    
    x_val = np.random.uniform(low = x_low , high = x_high)
    y_val =  np.random.uniform(low = y_low, high = y_high)
    
    if publish_goal_marker:
        
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = x_val
        goal_pose.pose.position.y = y_val
        pose_publisher.publish(goal_pose)
   
    return [x_val, y_val, theta]
      
rospy.init_node("demo", anonymous = True)
rospy.on_shutdown(shutdown_process)

amcl = AmclPose() #TODO - WE ARE ACTUALLY USING ODOMETRY AS AMCL, clean this up
# Setup clients
move_base = MoveBaseClient()
torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
head_action = PointHeadClient()
grasping_client = GraspingClient()
#    #        
gazebo_client = GazeboPoseMaster()
#    #    
pose_publisher =  RvizMarkerPublish()

##
#Move the base to be in front of the table
rospy.loginfo("Setting initial pose")   
#amcl.set_pose()
rospy.loginfo("Moving to table...")
if sample_random_nav_goal:
    nav_goal = sample_valid_navigation_goal()
    print('RANDOM GOAL')
else:
    nav_goal = default_nav_goal
    
#grasping_client.updateScene()
gazebo_client.set_pose_relative('coke_can', can_offset_x, can_offset_y, 0)
#
obj_pose_before = gazebo_client.get_pose('coke_can')
logger.update_log('Can Pose', obj_pose_before)

move_base.goto(nav_goal[0], nav_goal[1], nav_goal[2]) #unpack goal
#pdb.set_trace()
#rospy.sleep(3)

head_action.look_at_surroundings(pan_range = 45, tilt_range = 45) #build octomap

obj_pose = gazebo_client.get_pose()
grasping_client.pick(obj_pose)
#
logger.update_log('Success', gazebo_client.check_grasp_success())






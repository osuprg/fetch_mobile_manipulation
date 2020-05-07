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
from moveit_commander import Constraints
from moveit_msgs.msg import JointConstraint
        
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
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

sample_both_sides = (config.get(experiment_section_name, 'sample_both_sides') == 'True') #THIS will override y_low and high if true

x_low = float(config.get(experiment_section_name, 'sample_xlow')) + can_offset_x
x_high = float(config.get(experiment_section_name, 'sample_xhigh'))  + can_offset_x
y_low = float(config.get(experiment_section_name, 'sample_ylow'))  + can_offset_y
y_high = float(config.get(experiment_section_name, 'sample_yhigh'))  + can_offset_y
yaw = float(config.get(experiment_section_name, 'yaw'))

success_height = float(config.get(experiment_section_name, 'success_height'))
#default_nav_goal = [-0.45, 1.65, yaw]
default_nav_goal = [-0.45, 2.90, yaw]
default_nav_goal[0] += can_offset_x
default_nav_goal[1] += can_offset_y

log_directory = config.get(experiment_section_name, 'log_folder')

constrain_wrist_flex_joint = (config.get(experiment_section_name, 'constrained_wrist_flex_joint') == 'True')
#pdb.set_trace()
# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.clear_costmap_srv_name = '/move_base/clear_costmaps'
        self.costmap_clearer = rospy.ServiceProxy(self.clear_costmap_srv_name, Empty)
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
    
    
    def clear_costmap(self):
        
        rospy.wait_for_service(self.clear_costmap_srv_name)
        self.costmap_clearer.call()
        

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
        
        self.tilt_pan_head(pan = -pan_range, tilt = tilt_range)
        self.tilt_pan_head(pan = pan_range, tilt = tilt_range)
        self.tilt_pan_head(pan = 0, tilt = 0)
        

# Tools for grasping
class GraspingClient(object):

    def __init__(self, group="arm"):
        
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
        
        self.gazebo_client = GazeboPoseMaster()
        
        if constrain_wrist_flex_joint:
            self.fix_joint('wrist_flex_joint') #fix elbow joint

    def pick(self, obj_gazebo_pose):
         

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

         pose_publisher.publish(grasp_pose)
        
         self.group.set_pose_target(grasp_pose)

         logger.update_log('Arm Planning Start')
         
         plan = self.group.plan()
         logger.update_log('Arm Planning End')
         logger.update_log('Arm Planning Success', bool(plan.joint_trajectory.points))

         #log time after plan completed
         #log execution time start
         logger.update_log('Arm Execution Start Pose', amcl.get_pose())
         logger.update_log('Arm Execution Start')
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
         cartesian_servoing_success = self.move_gripper_linearly(grasp_pose_in_base, reduce_height_by = 0.0, avoid_collisions = False)

         logger.update_log('Cartesian Servoing Success', cartesian_servoing_success)
         time.sleep(0.25)
         cartesian_moving_down_success = self.move_gripper_linearly(grasp_pose_in_base, reduce_height_by = 0.1, avoid_collisions = False)
         logger.update_log('Cartesian Linear Success', cartesian_moving_down_success)
         logger.update_log('Arm Execution End')
         logger.update_log('Arm Execution End Pose', amcl.get_pose())
         time.sleep(0.5)
        
         self.gripper_open()
         self.gripper_close()
         time.sleep(0.5)
         self.move_gripper_linearly(grasp_pose_in_base, reduce_height_by = -0.2) #lift up


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
        
    def fix_joint(self, joint_name): #by default last joint remains at default pos
        
        try:
            joint_idx = self.group.get_active_joints().index(joint_name)
        except ValueError:
            raise
            
        c = Constraints()
        jc = JointConstraint()
        
        jc.joint_name = self.group.get_active_joints()[joint_idx]
        jc.position = self.group.get_current_joint_values()[joint_idx]
        jc.weight = 1.0
        jc.tolerance_above = 0.025
        jc.tolerance_below = 0.025
        
        c.joint_constraints.append(jc)
        
        self.group.set_path_constraints(c)
        
    def updateScene(self):
       
        # insert table to scene
        table_height = 0.47
        table_width = 0.8
        table_length = 0.4
        
        table_pose_in_world = self.gazebo_client.get_pose('coke_can')
        self.scene.attach_box("map", "table", 
                         table_pose_in_world, (table_length, table_width, table_height)
                        )
        
    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return
            
    def move_gripper_linearly(self, grasp_pose, reduce_height_by = 0.20, avoid_collisions = False, eef_step = 0.005): #computes cartesian path and goes down by depth m
        
        waypoints = []
        
        waypoints.append(grasp_pose.pose)
        target_pose = copy.deepcopy(grasp_pose)
        
        target_pose.pose.position.z -= reduce_height_by
        waypoints.append(target_pose.pose)
        
        trajectory, fraction = self.group.compute_cartesian_path(waypoints, eef_step, 0, avoid_collisions)
        cartesian_execute_success = self.group.execute(trajectory) #execute previously planned trajectory
        
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
      
rospy.init_node("demo", anonymous = True)
rospy.on_shutdown(shutdown_process)

amcl = AmclPose() #TODO - WE ARE ACTUALLY USING ODOMETRY AS AMCL, clean this up
# Setup clients
move_base = MoveBaseClient()
torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
head_action = PointHeadClient()
grasping_client = GraspingClient("arm")
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
    
gazebo_client.set_pose_relative('coke_can', can_offset_x, can_offset_y, 0)

move_base.clear_costmap()
#
obj_pose_before = gazebo_client.get_pose('coke_can')
logger.update_log('Can Pose', obj_pose_before)

move_base.goto(nav_goal[0], nav_goal[1], nav_goal[2]) #unpack goal

time.sleep(5)
head_action.look_at_surroundings(pan_range = 30, tilt_range = 45) #build octomap

obj_pose = gazebo_client.get_pose()
grasping_client.pick(obj_pose)
#
logger.update_log('Success', gazebo_client.check_grasp_success(height_threshold = success_height))

#Time to request to kill ourselves!
try: #To avoid error that service call didn't return we wrap in try catch
    rospy.wait_for_service('kill_launch')
    kill_us = rospy.ServiceProxy('kill_launch', Empty)
    kill_us.call()
except:
    pass







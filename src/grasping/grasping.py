#!/usr/bin/python2
# -*- coding: utf-8 -*-

import copy
import numpy as np
import sys
import time

from control_msgs.msg import GripperCommandGoal, GripperCommandAction
from geometry_msgs.msg import Quaternion, PoseStamped
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from moveit_commander import Constraints
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import MoveItErrorCodes
from std_srvs.srv import Empty
import actionlib
import moveit_commander
import rospy
import tf
import tf2_geometry_msgs
import tf2_ros

from gazebo_utils.pose_utils import GazeboPoseMaster
from rviz_utils.rviz_utils import RvizMarkerPublisher

class GraspingClient(object):

    def __init__(self, group="arm"):
        
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
        self.pose_publisher =  RvizMarkerPublisher()
        
    def pick(self, obj_gazebo_pose):
         
         self.pose_publisher.publish(obj_gazebo_pose)
         grasp_pose = PoseStamped()
         grasp_pose.pose.position = obj_gazebo_pose.pose.position

         quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) #rotation about z axis
         grasp_pose.pose.orientation = Quaternion(*quat)

         grasp_pose.header.stamp = rospy.Time.now()
         grasp_pose.header.frame_id = 'map'
         grasp_pose.pose.position.z+= self.gripper_height_above

         self.pose_publisher.publish(grasp_pose)
        
         self.group.set_pose_target(grasp_pose)
         plan = self.group.plan()
       
         arm_execution_success = self.group.execute(plan)
         #log execution time end
         time.sleep(1)
         if not plan.joint_trajectory.points:
             return

         cartesian_servoing_success = self.move_gripper_linearly(grasp_pose, delta_x = 0, delta_y = 0, delta_z = 0, avoid_collisions = False)

         time.sleep(0.25)
         cartesian_moving_down_success = self.move_gripper_linearly(grasp_pose, delta_x = 0, delta_y = 0, delta_z = -0.1, avoid_collisions = False)
         time.sleep(0.5)
        
         self.gripper_open()
         self.gripper_close()
         time.sleep(0.5)
         self.move_gripper_linearly(grasp_pose, delta_z = 0.2) #lift up

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
            
    def move_gripper_linearly(self, current_pose, delta_x = None, delta_y = None, delta_z = None, avoid_collisions = False, eef_step = 0.005): #computes cartesian path and goes down by depth m
            
        waypoints = []
        
        old_frame = self.group.get_pose_reference_frame() #make backup of the original frame
        self.group.set_pose_reference_frame(current_pose.header.frame_id) #cartesian trajectory plans in this frame
        waypoints.append(current_pose.pose) #our current pose
        target_pose = copy.deepcopy(current_pose)
        
        if delta_x:
            target_pose.pose.position.x += delta_x
        if delta_y:
            target_pose.pose.position.y += delta_y
        if delta_z:
            target_pose.pose.position.z += delta_z
            
        waypoints.append(target_pose.pose)
        
        trajectory, fraction = self.group.compute_cartesian_path(waypoints, eef_step, 0, avoid_collisions)
        cartesian_execute_success = self.group.execute(trajectory) #execute previously planned trajectory
        
        self.group.set_pose_reference_frame(old_frame) #reset back to old planning frame
        
        return cartesian_execute_success
        
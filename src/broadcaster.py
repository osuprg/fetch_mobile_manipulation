#!/usr/bin/env python  
import rospy

import tf
from nav_msgs.msg import Odometry
import tf_conversions
from geometry_msgs.msg import Transform



br = tf.TransformBroadcaster()

def publish_odom(msg):
    
    
    pose = msg.pose.pose
    br.sendTransform((pose.position.x, pose.position.y, pose.position.z), 
                     (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

if __name__ == '__main__':
    rospy.init_node('odom_fake_publisher')
    rospy.Subscriber('odom_fake', 
                     Odometry,
                     publish_odom)
                    
    rospy.spin()

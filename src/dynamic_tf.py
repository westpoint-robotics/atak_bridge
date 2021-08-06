#!/usr/bin/env python

import os
import sys
import time
import uuid
import socket
import numpy as np

import rospy
import rospkg
import tf
import tf2_ros
import geometry_msgs.msg

import tf.msg

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3Stamped
#from arl_nav_msgs.msg import GotoRegionActionGoal # Used to publish target location as a goto goal
from nav_msgs.msg import Odometry


class DynTFBroadcaster:
    
    def __init__(self):
        self.robot_name          = rospy.get_param('~robot_name', 'husky')
        self.baselink_frame      = rospy.get_param('~baselink_frame', 'baselink_frame') 
        self.global_frame        = rospy.get_param('~global_frame', 'utm')                
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage)

        rospy.Subscriber('/' + self.robot_name + '/odom', Odometry, self.dynamic_tf)
        
    def dynamic_tf(self, data):
        t = geometry_msgs.msg.TransformStamped()
        
        t.header = data.header
        # set the time as ROS time
        t.header.frame_id = 'map'
        t.child_frame_id = self.baselink_frame

        t.transform.translation.x = data.pose.pose.position.x
        t.transform.translation.y = data.pose.pose.position.y
        t.transform.translation.z = data.pose.pose.position.z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        tfm = tf.msg.tfMessage([t])
        self.pub_tf.publish(tfm)
            
if __name__ == '__main__':
    rospy.init_node('baselink_tf_broadcaster')
    DynTFBroadcaster()
    rospy.spin()

        
        
        

        

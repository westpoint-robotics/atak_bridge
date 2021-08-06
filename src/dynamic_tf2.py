#!/usr/bin/env python

import os
import sys
import time
import socket
import numpy as np

import logging

import rospy
import rospkg
import tf2_ros
import geometry_msgs.msg

from nav_msgs.msg import Odometry


class DynTFBroadcaster:
    
    def __init__(self):
        self.robot_name          = rospy.get_param('~robot_name', 'husky')
        self.baselink_frame      = rospy.get_param('~baselink_frame', 'baselink_frame') 
        self.global_frame        = rospy.get_param('~global_frame', 'utm')                

        rospy.Subscriber('/' + self.robot_name + '/odom', Odometry, self.dynamic_tf)
        
    def dynamic_tf(self, data):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header = data.header
        t.header.frame_id = 'map'
        t.child_frame_id = self.baselink_frame
        
        t.transform.translation.x = data.pose.pose.position.x
        t.transform.translation.y = data.pose.pose.position.y
        t.transform.translation.z = data.pose.pose.position.z
#        q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform(t)
    
            
if __name__ == '__main__':
    rospy.init_node('tf2_baselink_broadcaster')
    DynTFBroadcaster()
    rospy.spin()


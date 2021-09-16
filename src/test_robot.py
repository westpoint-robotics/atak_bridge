#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

'''
  <node type="static_transform_publisher" pkg="tf2_ros" name="dummy2_utm_frame" args="587361 4582574 0 0 0 1.571 utm $(arg name)/map" />

rosrun tf2_ros static_transform_publisher 587361 4582574 0 0 0 1.571 utm husky/map
'''

import os
import sys
import time
import uuid
import socket
import numpy as np
import math

import json
import xml.etree.ElementTree as ET
from  xml.dom.minidom import parseString
import logging

import rospy
import rospkg
import tf

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose
from atak_bridge.msg import PoseDescription, PoseDescriptionStamped, PoseDescriptionArray

from LatLongUTMconversion import LLtoUTM, UTMtoLL

#TODO Investigate handling of TAK messages arriving faster than being processed by this code.
#TODO Find a sufficient answer to send duplicate targets. Seeing 2 tgts and reporting one, and the other way around.

class TestRobot:
    """A class used to communication between an ATAK and robots"""

    def __init__(self):
        self.robot_name          = rospy.get_param('~name', "husky")
        self.baselink_frame      = rospy.get_param('~baselink_frame', 'base_link')
        self.global_frame        = rospy.get_param('~global_frame', 'utm')
        self.zone='18T'
        self.targets = [("people",(10,25)),("car",(-10,25))]
        self.vis_pub = rospy.Publisher("goto_marker", Marker, queue_size=10)        
        self.obj_pub = rospy.Publisher("object_location", PoseDescriptionArray, queue_size=10)
        rospy.Subscriber("goto_goal", PoseDescriptionStamped, self.goto_cb)
                
        rospy.loginfo("Started ATAK Bridge Test Robot:\n\t\Robot Name: %s"
                    %(self.robot_name))

    def goto_cb(self, data):
        marker = Marker()
        marker.header.frame_id = "utm"           
        marker.header.stamp = rospy.Time.now()
        marker.ns = "husky"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = data.pose.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0 
        marker.color.r = 1.0
        self.vis_pub.publish( marker )   

    def publish_targets(self):
        trgt_msg = PoseDescriptionArray()
        for target in self.targets:
            # rospy.loginfo("TARGET: %s" % target[0])
            trgt_msg.header.stamp = rospy.Time.now()
            trgt_msg.header.frame_id = 'odom'
            pd = PoseDescription()
            pd.description.data = target[0]
            pd.pose.position.x = target[1][0]
            pd.pose.position.y = target[1][1]
            trgt_msg.pose_list.append(pd)
        self.obj_pub.publish(trgt_msg)
 
if __name__ == '__main__':
    try:
        rospy.init_node("test_robot")
        sim_robot = TestRobot()
        br = tf.TransformBroadcaster()
        loop_hz = 20
        rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the timeout.
        x = 0
        incr = 1
        while not rospy.is_shutdown():
            # Using sine wiave to simulate robot motion
            y = math.sin(x/100.0)
            x += incr
            br.sendTransform((x/100.0*5, y*8, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 1.57),
                             rospy.Time.now(),
                             "base_link",
                             "odom")
            if (abs(x) > 1000):
                incr = incr * -1
                
            sim_robot.publish_targets()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("EXCEPTION THROWN")

        pass    

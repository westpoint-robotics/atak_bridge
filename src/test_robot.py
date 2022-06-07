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
from geometry_msgs.msg import PoseStamped, Pose, Point
from atak_bridge.msg import PoseDescription, PoseDescriptionStamped, PoseDescriptionArray

from LatLongUTMconversion import LLtoUTM, UTMtoLL

#TODO Investigate handling of TAK messages arriving faster than being processed by this code.
#TODO Find a sufficient answer to send duplicate targets. Seeing 2 tgts and reporting one, and the other way around.

class TestRobot:
    """A class used to communication between an ATAK and robots"""

    def __init__(self):
        self.robot_name          = rospy.get_param('~name', "husky")
        self.baselink_frame      = rospy.get_param('~baselink_frame', 'base_link')
        self.map_frame           = rospy.get_param('~map_frame', 'map') 
        self.zone='18T'
        self.targets = [("People",(10,25)),("Car",(-10,25)),("Rock",(-10,-25))] # simulated targets
        self.target_list = ['People','Car'] # List of objects of interest
        self.vis_pub = rospy.Publisher("goto_marker", Marker, queue_size=10)        
        self.obj_pub = rospy.Publisher("object_location", PoseDescriptionArray, queue_size=10)
        self.point = Point() 
        self.incr = 1 
        rospy.Subscriber("goto_goal", PoseDescriptionStamped, self.goto_cb)
                
        rospy.loginfo("Started ATAK Bridge Test Robot:\n\t\Robot Name: %s"
                    %(self.robot_name))

    # This callback publishes a marker at the goto location when the robot recieves a 
    # goto message from atak_bridge.
    def goto_cb(self, data):
        marker = Marker()
        marker.header.frame_id = self.map_frame           
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.robot_name
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
            if (target[0] in self.target_list):
                #rospy.loginfo("TARGET: %s" % target[0])
                trgt_msg.header.stamp = rospy.Time.now()
                trgt_msg.header.frame_id = self.map_frame
                pd = PoseDescription()
                pd.description.data = target[0]
                pd.pose.position.x = target[1][0]
                pd.pose.position.y = target[1][1]
                trgt_msg.pose_list.append(pd)
        self.obj_pub.publish(trgt_msg)

    def sim_robot_move(self):
        # Using sine wave to simulate robot motion
        self.point.y = math.sin(self.point.x/100.0)
        self.point.x += self.incr
        br.sendTransform((self.point.x/100.0*5, self.point.y*8, 0),
                            tf.transformations.quaternion_from_euler(0, 0, 1.57),
                            rospy.Time.now(),
                            self.baselink_frame,
                            self.map_frame)
        if (abs(self.point.x) > 1000):
            self.incr = self.incr * -1
 
if __name__ == '__main__':
    try:
        rospy.init_node("test_robot")
        sim_robot = TestRobot()
        br = tf.TransformBroadcaster()
        loop_hz = 20
        rate = rospy.Rate(loop_hz) 
        while not rospy.is_shutdown():
            sim_robot.sim_robot_move()
            #sim_robot.publish_targets()
            rate.sleep()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("EXCEPTION THROWN")

        pass    

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
from atak_bridge.msg import PoseDescriptionHeader, PoseDescriptionArray

from takpak.mkcot import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL

#TODO Investigate handling of TAK messages arriving faster than being processed by this code.
#TODO Find a sufficient answer to send duplicate targets. Seeing 2 tgts and reporting one, and the other way around.

class TestRobot:
    """A class used to communication between an ATAK and robots"""

    def __init__(self):
        self.robot_name          = rospy.get_param('~name', "husky")
        self.my_team_name        = rospy.get_param('~team_name', 'Cyan') # Use one from ATAK which are colors
        self.my_team_role        = rospy.get_param('~team_role', 'Team Member') # Use one from ATAK
        self.tak_ip              = rospy.get_param('~tak_ip', '127.0.0.1') 
        self.tak_port            = rospy.get_param('~tak_port', '8088') # Port for TCP un-encrypted connection
        self.my_callsign         = rospy.get_param('~callsign', 'default_callsign')
        self.baselink_frame      = rospy.get_param('~baselink_frame', 'base_link')
        self.global_frame        = rospy.get_param('~global_frame', 'utm')
        self.my_uid              = 'husky1'
        self.zone='18T'
        self.takmsg_tree = ''
        self.target_list = ["people"]#["car", "Vehicle"]
        self.vis_pub = rospy.Publisher("goto_marker", Marker, queue_size=10)        
        self.obj_pub = rospy.Publisher("object_location", PoseDescriptionArray, queue_size=10)
        rospy.Subscriber("goto_goal", PoseDescriptionHeader, self.goto_cb)
                
        rospy.loginfo("Started ATAK Bridge Test Robot:\n\t\tCallsign: %s\n\t\tUID: %s\n\t\tTeam name: %s"
                    %(self.my_callsign,self.my_uid,self.my_team_name))
        self.takserver = takcot() #TODO add a timeout and exit condition

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
 
if __name__ == '__main__':
    try:
        rospy.init_node("test_robot")
        bridge = TestRobot()
        br = tf.TransformBroadcaster()
        loop_hz = 20
        rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the timeout.
        x = 0
        incr = 1
        while not rospy.is_shutdown():
            y = math.sin(x/100.0)
            x += incr
            br.sendTransform((x/100.0*5, y*8, 0),
                             tf.transformations.quaternion_from_euler(0, 0, 1.57),
                             rospy.Time.now(),
                             "base_link",
                             "odom")
            if (abs(x) > 1000):
                incr = incr * -1
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass    

#!/usr/bin/env python
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

'''
  <node type="static_transform_publisher" pkg="tf2_ros" name="dummy2_utm_frame" args="587361 4582574 0 0 1.571 utm $(arg name)/map" />

rosrun tf2_ros static_transform_publisher 587361 4582574 0 0 0 1.571 utm husky/map
'''

import os
import sys
import time
import rospy
import tf

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Int16
from std_msgs.msg import Header, ColorRGBA


rospy.init_node('readings_markers', anonymous=True)
marker_publisher = rospy.Publisher('reading_marker', Marker, queue_size=10)
tf1_listener = tf.TransformListener()
tf1_listener.waitForTransform('map', 'front_laser', rospy.Time(0), rospy.Duration(35.0))
marker_id = 0

def readings_cb(data):
    global marker_id
    marker_id+=1
    reading = data.data
    rospy.loginfo("REcieved a reading of: %d",reading)
    text=str(reading)
    tf1_listener.waitForTransform('map', 'front_laser', rospy.Time(0), rospy.Duration(35.0))
    (trans,rot) = tf1_listener.lookupTransform('map', 'front_laser', rospy.Time(0)) 
    rdg_pose=Pose()
    rdg_pose.position.x=trans[0]
    rdg_pose.position.y=trans[1]
    rdg_pose.position.z=trans[2]
    rdg_pose.orientation.x=rot[0]
    rdg_pose.orientation.y=rot[1]
    rdg_pose.orientation.z=rot[2]
    rdg_pose.orientation.w=rot[3]
    hdr=Header()
    hdr.frame_id = 'map'
    hdr.stamp = rospy.Time.now() 
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=marker_id,
                action = Marker.ADD,
                lifetime=rospy.Duration(5.5),
                pose=rdg_pose,
                header=hdr,
                scale=Vector3(0.25, 0.25, 0.25),
                color=ColorRGBA(1.0, 0.0, 0.0, 1.0),
                text=text)
    marker_publisher.publish(marker)

rospy.Subscriber("random_num", Int16, readings_cb)            
rospy.spin()


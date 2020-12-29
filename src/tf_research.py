#!/usr/bin/env python
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

import rospy
import tf

# Setup ROS node
rospy.init_node('tf_research', anonymous=True)

tf1_listener = tf.TransformListener()
#tf1_listener.waitForTransform("uas1/map", "utm", rospy.Time(0), rospy.Duration(5.0))
rospy.sleep(5)
frames = tf1_listener.allFramesAsString()
frames_sorted = []


for item in frames.split("\n"):
    line = item.split(" ")
    if(len(line) > 4):
        out_item = ("%20s --->> %s" %(line[5][:-1],line[1]))
        frames_sorted.append(out_item)

frames_sorted = sorted(frames_sorted)
for item in frames_sorted:
        print(item)
        
'''
/home/user1/catkin_ws/src/atak_bridge/src/tf_research.py

rosparam get /uav1/mavros/local_position/tf/send
false
user1@rrc-prec01:~/.../src$ rosparam get /uav1/mavros/global_position/tf/send
false
      uav1/base_link --->> uav1/base_link_inertia
            uav1/map --->> uav1/map_ned
           uav1/odom --->> uav1/odom_ned
                 utm --->> uav1/odom
      uav1/base_link --->> uav1/base_link_frd
                uav1 --->> uav1/IMU
                uav1 --->> uav1/MonoCamera_Down
                uav1 --->> uav1/MonoCamera_Right
                uav1 --->> uav1/RGBCamera_Mid
                uav1 --->> uav1/MonoCamera_Left
               world --->> ground_truth/uav1/base
bas
ground_truth/uav1/base
uav1/base_link
=====================================================================================================
rosparam get /uav1/mavros/local_position/tf/send
true
rosparam get /uav1/mavros/global_position/tf/send
false
      uav1/base_link --->> uav1/base_link_inertia
           uav1/odom --->> uav1/base_link
            uav1/map --->> uav1/map_ned
           uav1/odom --->> uav1/odom_ned
                 utm --->> uav1/odom
      uav1/base_link --->> uav1/base_link_frd
                uav1 --->> uav1/IMU
                uav1 --->> uav1/MonoCamera_Down
                uav1 --->> uav1/MonoCamera_Right
                uav1 --->> uav1/RGBCamera_Mid
                uav1 --->> uav1/MonoCamera_Left
               world --->> ground_truth/uav1/base

=====================================================================================================
=====================================================================================================
=====================================================================================================
[ WARN] [1608937578.722527523, 111.584000000]: Could not obtain transform from uav1/base_link to uav1/odom. Error was Could not find a connection between 'uav1/odom' and 'uav1/base_link' because they are not part of the same tree.Tf has two or more unconnected trees.

[ WARN] [1608937579.529675333, 112.376000000]: "local_origin" passed to lookupTransform argument target_frame does not exist. 

[ WARN] [1608937578.371354408, 111.384000000]: Could not obtain uav1/odom->uav1/base_link transform. Will not remove offset of navsat device from robot's origin.


'''        


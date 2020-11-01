#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Vector3Stamped
from arl_nav_msgs.msg import GotoRegionActionGoal


start_latitude = 41.393850 # Soccer field
start_longitude = -73.953674
start_altitude = 10
#UTM Zone = 18T
#UTM Easting = 587472.34
#UTM Northing = 4583007.81
#LLtoUTM(23, 41.393850,-73.953674)
#UTMtoLL(23, nrt, est, '18T')

(zone,start_utm_e,start_utm_n) = LLtoUTM(23, start_latitude, start_longitude)
start_utm_alt = start_altitude

rospy.init_node('goto_target', anonymous=True)
pub = rospy.Publisher('/warty/goto_region/goal', GotoRegionActionGoal, queue_size=10)
rospy.Subscriber("/atak/atak_tgt", Vector3Stamped, target_cb)

def target_cb(data):
    (zone,tgt_utm_e,tgt_utm_n) = LLtoUTM(23, data.x, data.y)
    tgt_pos_x = tgt_utm_e - start_utm_e
    tgt_pos_y = tgt_utm_n - start_utm_n
    rospy.loginfo('Target of: %f, %f'tgt_pos_x,tgt_pos_y)
    tgt_utm_x = current_odom.pose.pose.position.x + start_utm_x
    tgt_utm_y = current_odom.pose.pose.position.y + start_utm_y

    msg = GotoRegionActionGoal()
    msg.header.stamp = rospy.Time.now()
    goal_id.header.stamp = rospy.Time.now()
    goal_id.id = "ATAK GOTO"
    goal.region_center.header.stamp = data.header.stamp
    goal.region_center.header.frame_id = "warty/map"
    goal.region_center.pose.position.x = tgt_pos_y
    goal.region_center.pose.position.y = tgt_utm_y
    goal.radius = 0.75
    goal.angle_threshold = 3.14 # Do not care which direction it faces

rate = rospy.Rate(1) # 10hz
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    #msg.status.status = 0
    #msg.status.service = 0
    msg.latitude = msg.latitude + 0.0005
    msg.longitude = msg.longitude + 0.0005 
    #msg.position_covariance
    #msg.position_covariance_type
    #hello_str = "hello world %s" % rospy.get_time()
    #rospy.loginfo(hello_str)
    pub.publish(msg)
    rate.sleep()


        
'''



'''

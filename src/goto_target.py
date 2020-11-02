#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Vector3Stamped
from arl_nav_msgs.msg import GotoRegionActionGoal
from LatLongUTMconversion import LLtoUTM, UTMtoLL

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

rospy.init_node('atak_target', anonymous=True)
pub = rospy.Publisher('/warty/goto_region/goal', GotoRegionActionGoal, queue_size=10)

def target_cb(data):
    (zone,tgt_utm_e,tgt_utm_n) = LLtoUTM(23, data.vector.x, data.vector.y)
    tgt_pos_x = tgt_utm_e - start_utm_e
    tgt_pos_y = tgt_utm_n - start_utm_n
    rospy.loginfo('Target of: %f, %f' %(tgt_pos_x,tgt_pos_y))
    tgt_utm_x = tgt_pos_x + start_utm_e
    tgt_utm_y = tgt_pos_y + start_utm_n

    msg = GotoRegionActionGoal()
    msg.header.stamp = rospy.Time.now()
    msg.goal_id.stamp = rospy.Time.now()
    msg.goal_id.id = "ATAK GOTO"
    msg.goal.region_center.header.stamp = data.header.stamp
    msg.goal.region_center.header.frame_id = "warty/map"
    msg.goal.region_center.pose.position.x = tgt_pos_y
    msg.goal.region_center.pose.position.y = tgt_utm_y
    msg.goal.radius = 0.75
    msg.goal.angle_threshold = 3.14 # Do not care which direction it faces
    pub.publish(msg)
    
rospy.Subscriber("atak_tgt", Vector3Stamped, target_cb)

rospy.spin() # Just spin here and react to recieving a target from the subscriber


        
'''
root@rrc-prec01:/home/user1/phoenix-r1# rosmsg show arl_nav_msgs/GotoRegionActionGoal 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalID goal_id
  time stamp
  string id
arl_nav_msgs/GotoRegionGoal goal
  geometry_msgs/PoseStamped region_center
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  float64 radius
  float64 angle_threshold
  float64 absolute_duration_limit
  float64 relative_duration_limit
  geometry_msgs/Polygon boundary
    geometry_msgs/Point32[] points
      float32 x
      float32 y
      float32 z



'''

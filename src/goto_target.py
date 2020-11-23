#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Vector3Stamped
from arl_nav_msgs.msg import GotoRegionActionGoal
from LatLongUTMconversion import LLtoUTM, UTMtoLL

rospy.init_node('atak_target', anonymous=True)

# start_latitude = 41.393850 # Soccer field
# start_longitude = -73.953674
# start_altitude = 10
start_latitude = 41.39058 # Rivercourts
start_longitude = -73.95323
start_altitude = 10
start_heading = 0 # This is due west
robot_name = "warty"
#UTM Zone = 18T
#UTM Easting = 587472.34
#UTM Northing = 4583007.81
#LLtoUTM(23, 41.393850,-73.953674)
#UTMtoLL(23, nrt, est, '18T')
start_latitude = rospy.get_param('~start_latitude', start_latitude)
start_longitude = rospy.get_param('~start_longitude', start_longitude) 
start_altitude = rospy.get_param('~start_altitude', start_altitude)
start_heading = rospy.get_param('~start_heading', start_heading)
robot_name = rospy.get_param('~name', "warty")
goal_topic="/"+robot_name+"/goto_region/goal"
rospy.loginfo('goto_target node is subscribing to the topic of: %s' %(goal_topic))

(zone,start_utm_e,start_utm_n) = LLtoUTM(23, start_latitude, start_longitude)
start_utm_alt = start_altitude

pub = rospy.Publisher(goal_topic, GotoRegionActionGoal, queue_size=10)

def target_cb(data):
    (zone,tgt_utm_e,tgt_utm_n) = LLtoUTM(23, data.vector.x, data.vector.y)
    tgt_pos_x = (tgt_utm_e - start_utm_e)
    tgt_pos_y = (tgt_utm_n - start_utm_n)
    rospy.loginfo('Target of: %f, %f' %(tgt_pos_x,tgt_pos_y))

    msg = GotoRegionActionGoal()
    msg.header.stamp = rospy.Time.now()
    msg.goal_id.stamp = rospy.Time.now()
    msg.goal_id.id = "ATAK GOTO"
    msg.goal.region_center.header.stamp = data.header.stamp
    msg.goal.region_center.header.frame_id = "husky/map"
    msg.goal.region_center.pose.position.x = tgt_pos_x
    msg.goal.region_center.pose.position.y = tgt_pos_y
    msg.goal.region_center.pose.orientation.z = 0.0985357937255
    msg.goal.region_center.pose.orientation.w = 0.995133507302
    msg.goal.radius = 3.75
    msg.goal.angle_threshold = 3.108 
    pub.publish(msg)
    
rospy.Subscriber("atak_tgt", Vector3Stamped, target_cb)

rospy.spin() # Just spin here and react to receiving a target from the subscriber


        
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

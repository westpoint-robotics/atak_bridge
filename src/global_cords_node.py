#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from LatLongUTMconversion import LLtoUTM, UTMtoLL


start_latitude = 41.393850 # Soccer field
start_longitude = -73.953674
start_altitude = 10
#UTM Zone = 18T
#UTM Easting = 587472.34
#UTM Northing = 4583007.81

#LLtoUTM(23, 41.393850,-73.953674)
#UTMtoLL(23, nrt, est, '18T')

(zone,start_utm_x,start_utm_y) = LLtoUTM(23, start_latitude, start_longitude)
start_utm_alt = start_altitude

current_odom = Odometry()

def odom_cb(data):
    global current_odom
    current_odom = data
    #rospy.loginfo("calback firint, odom.x %f, odom.y %f "%(current_odom.pose.pose.position.x, current_odom.pose.pose.position.y))

def main():
    global current_odom
    pub = rospy.Publisher('atak_fix', NavSatFix, queue_size=10)
    rospy.init_node('global_coords', anonymous=True)
    rospy.Subscriber("/warty/odom", Odometry, odom_cb)
    msg = NavSatFix()
    msg.latitude = 41.393850 # Soccer field
    msg.longitude = -73.953674
    msg.altitude = 10
    
    rate = rospy.Rate(2) 
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        crnt_utm_x = current_odom.pose.pose.position.x + start_utm_x
        crnt_utm_y = current_odom.pose.pose.position.y + start_utm_y
        (msg.latitude,msg.longitude) = UTMtoLL(23, crnt_utm_y, crnt_utm_x, zone) # 23 is WGS-84. 
        
        #msg.status.status = 0
        #msg.status.service = 0
        #msg.latitude = msg.latitude + 0.0005
        #msg.longitude = msg.longitude + 0.0005 
        #msg.position_covariance
        #msg.position_covariance_type
        pub.publish(msg)
        #rospy.loginfo("lat: %f, lon %f, odom.x %f, odom.y %f" %(msg.latitude,msg.longitude,current_odom.pose.pose.position.x, current_odom.pose.pose.position.y))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
        
'''user1@rrc-prec01:~/catkin_ws$ rosmsg show sensor_msgs/NavSatFix
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
sensor_msgs/NavSatStatus status
  int8 STATUS_NO_FIX=-1
  int8 STATUS_FIX=0
  int8 STATUS_SBAS_FIX=1
  int8 STATUS_GBAS_FIX=2
  uint16 SERVICE_GPS=1
  uint16 SERVICE_GLONASS=2
  uint16 SERVICE_COMPASS=4
  uint16 SERVICE_GALILEO=8
  int8 status
  uint16 service
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type

'''

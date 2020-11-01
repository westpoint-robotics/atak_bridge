#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

def talker():
    pub = rospy.Publisher('atak_fix', NavSatFix, queue_size=10)
    rospy.init_node('gps_sim', anonymous=True)
    msg = NavSatFix()
    msg.latitude = 41.393850 # Soccer field
    msg.longitude = -73.953674
    msg.altitude = 10
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

if __name__ == '__main__':
    try:
        talker()
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

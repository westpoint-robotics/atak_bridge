#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from LatLongUTMconversion import LLtoUTM, UTMtoLL


current_odom = Odometry()

def odom_cb(data):
    global current_odom
    current_odom = data
    #rospy.loginfo("calback firint, odom.x %f, odom.y %f "%(current_odom.pose.pose.position.x, current_odom.pose.pose.position.y))

def main():
    global current_odom
    rospy.init_node('global_coords', anonymous=True)

    # start_latitude = 41.393850 # Soccer field
    # start_longitude = -73.953674
    # start_altitude = 10
    #UTM Zone = 18T
    #UTM Easting = 587472.34
    #UTM Northing = 4583007.81
    start_latitude = 41.39058 # Rivercourts
    start_longitude = -73.95323
    start_altitude = 10
    start_heading = 0 # This is due west
    # UTM found using https://www.latlong.net/lat-long-utm.html
    #UTM Zone = 18T
    #UTM Easting = 587513.84
    #UTM Northing = 4582645.22
    robot_name = "warty"


    #LLtoUTM(23, 41.393850,-73.953674)
    #UTMtoLL(23, nrt, est, '18T')
    start_latitude = rospy.get_param('~start_latitude', start_latitude)
    start_longitude = rospy.get_param('~start_longitude', start_longitude)
    start_altitude = rospy.get_param('~start_altitude', start_altitude)
    start_heading = rospy.get_param('~start_heading', start_heading)

    robot_name = rospy.get_param('~name', robot_name)
    odom_topic="/"+robot_name+"/odom"
    rospy.loginfo('global_cords node is subscribing to the topic of: %s' %(odom_topic))

    # LLtoUTM returns zone,easting, northing.
    (zone,start_utm_e,start_utm_n) = LLtoUTM(23, start_latitude, start_longitude)

    pub = rospy.Publisher('atak_fix', NavSatFix, queue_size=10)
  
    rospy.Subscriber(odom_topic, Odometry, odom_cb)
    msg = NavSatFix()
    msg.latitude = start_latitude
    msg.longitude = start_longitude
    msg.altitude = start_altitude
    
    rate = rospy.Rate(2) 
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        # Using REP105, align x with east and y with north.
        crnt_utm_e = start_utm_e + current_odom.pose.pose.position.x
        crnt_utm_n = start_utm_n + current_odom.pose.pose.position.y 
        # UTMtoLL expects Ellipsoid, Northing, Easting, zone. It returns lat, long.
        (msg.latitude,msg.longitude) = UTMtoLL(23, crnt_utm_n, crnt_utm_e, zone) # 23 is WGS-84. 
        
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

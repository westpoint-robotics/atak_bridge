#!/usr/bin/env python
#TODO Validate this with a real robot. Conversion of orientation a position.

import rospy
from tf2_ros import StaticTransformBroadcaster, TransformStamped
from LatLongUTMconversion import LLtoUTM, UTMtoLL
from tf.transformations import euler_from_quaternion, quaternion_from_euler


rospy.init_node('utm_2_map', anonymous=True)
tf_broadcaster = StaticTransformBroadcaster()
static_transformStamped = TransformStamped()

default_latitude = 41.39058 # Rivercourts
default_longitude = -73.95323
default_altitude = 10
default_heading = 0 #

start_latitude = rospy.get_param('~start_latitude', default_latitude)
start_longitude = rospy.get_param('~start_longitude', default_longitude)
start_altitude = rospy.get_param('~start_altitude', default_altitude)
start_heading_mag = rospy.get_param('~start_heading', default_heading) #In degrees magnetic north

# Convert start Lat/Long to UTM
(zone,start_utm_e,start_utm_n) = LLtoUTM(23, start_latitude, start_longitude)

# Rotate 90 degrees to change from 0 degrees being N to bing E.
if start_heading_mag >= 90:
    start_heading_mag = start_heading_mag - 90
else:
    start_heading_mag = start_heading_mag + 270
# Convert to radians
start_heading_rad = start_heading_mag / 180 * 3.14159
# Convert to +/- 3.14
if start_heading_rad > 3.14159:
    start_heading_rad = start_heading_rad - (3.14159*2)
# quaternion_from_euler(r,p,y)
q = quaternion_from_euler(0, 0, (start_heading_rad*-1))

static_transformStamped.header.stamp = rospy.Time.now()
static_transformStamped.header.frame_id = 'utm'
#static_transformStamped.child_frame_id = 'husky/map'
static_transformStamped.child_frame_id = 'experiment_frame'
static_transformStamped.transform.translation.x = start_utm_e
static_transformStamped.transform.translation.y = start_utm_n
static_transformStamped.transform.translation.z = start_altitude
static_transformStamped.transform.rotation.x = q[0]
static_transformStamped.transform.rotation.y = q[1]
static_transformStamped.transform.rotation.z = q[2]
static_transformStamped.transform.rotation.w = q[3]

tf_broadcaster.sendTransform(static_transformStamped)
rospy.spin()


'''
For NED type IMUs, the orientation of the world frame is x-north, y-east, z-down, relative to magnetic north.
For ENU type IMUs, the orientation of the world frame is x-east, y-north, z-up, relative to magnetic north.


coords to thayer hall: 41.38997729961688, -73.95504555359662

'''

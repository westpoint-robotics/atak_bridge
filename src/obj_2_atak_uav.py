#!/usr/bin/env python
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

import os
import sys
import time
import uuid
import socket
import json
import xml.etree.ElementTree as ET
from  xml.dom.minidom import parseString
import logging

import rospy
import tf
from visualization_msgs.msg import MarkerArray, Marker
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped

from takpak.mkcot import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL


node_name='obj_2_map'
zone='18T'

# Setup ROS node
rospy.init_node(node_name, anonymous=True)

robot_name = rospy.get_param('~name', "warty")

tf1_listener = tf.TransformListener()   
tf1_listener.waitForTransform("uav1/map", "utm", rospy.Time(0), rospy.Duration(5.0)) 

# Setup ATAK identity of this node 
my_callsign = rospy.get_param('~callsign', 'uas1')
if rospy.has_param('~uid'):
    my_uid = rospy.get_param('~uid')
else:
    my_uid = str(socket.getfqdn()) + "-" + str(uuid.uuid1())[-12:]
my_team_name = rospy.get_param('~team_name', 'Default Team')   
my_team_role = rospy.get_param('~team_role', 'Default Team Role')
tak_ip = rospy.get_param('~tak_ip', '127.0.0.1') 
tak_port = rospy.get_param('~tak_port', '8088') 
rospy.loginfo("my_callsign=%s, my_uid =%s, my_team_name =%s, my_uid =%s" %(my_callsign,my_uid,my_team_name,my_team_role))

# Start ATAK client
rospy.loginfo("============ Node: %s is connecting to  TAK Server at %s:%s===============" %(node_name,tak_ip,tak_port))
rospy.loginfo("==== If the code appears to freeze at this point, then it is likely the server is not reachable  =====")
takserver = takcot() #TODO add a timeout and exit condition
try:
    tasksock = takserver.open(tak_ip,tak_port)
    takserver.flush()    
except:
    rospy.logerr("Failed to connect to the TAK Server")
    exit()
    
# Send a ping and check for answer
connect_xml = (mkcot.mkcot(cot_type="t", cot_how="h-g-i-g-o", cot_callsign=my_callsign, cot_id=my_uid, team_name=my_team_name, team_role=my_team_role)).decode('utf-8')
my_xml = parseString(str(connect_xml.replace("\n",""))).toprettyxml()
tree = ET.fromstring(my_xml)
xml_callsign = tree.find("./detail/contact").attrib['callsign']
rospy.loginfo("SUCCESSFUL Conection with the server. The server believes my callsign is: %s" %(xml_callsign))
    
target_list = ["car","boat"]

def object_location_cb(data):
    global takserver
    global zone
    for detection in data.detections: 
        for result in detection.results:
            r_pose = result.pose.pose.position
            if result.id in target_list:
                obj_pose_utm_stamped = PoseStamped()
                obj_pose_utm_stamped.header = detection.header                        
                obj_pose_utm_stamped.pose = result.pose.pose 
                obj_pose_utm = tf1_listener.transformPose("utm", obj_pose_utm_stamped)
                (obj_latitude,obj_longitude) = UTMtoLL(23, obj_pose_utm.pose.position.y, obj_pose_utm.pose.position.x, zone) # 23 is WGS-84. 
                rospy.loginfo('ID: %s, Score: %.2f, Location: %.3f, %.3f, %.3f -- LatLong: %.5f, %.5f -- UTM: %.3f, %.3f, %.3f' %
                        (result.id,result.score, r_pose.x, r_pose.y,r_pose.z, obj_latitude, obj_longitude,
                        obj_pose_utm.pose.position.x, obj_pose_utm.pose.position.y,obj_pose_utm.pose.position.z))
                takserver.send(mkcot.mkcot(cot_identity="neutral", 
                    cot_stale = 1, 
                    #cot_dimensionlon="land-unit",
                    cot_type="a-f-G-M-F-Q",
                    #cot_type="a-f-G-U-C", 
                    cot_how="m-g", 
                    #cot_callsign=item.ns, 
                    cot_id="object", 
                    team_name="deteczone='18T'tor", 
                    team_role="obj detector",
                    cot_lat=obj_latitude,
                    cot_lon=obj_longitude ))                       

def main():
    rospy.Subscriber("/uav1/detection_localization/detections/out/local", Detection2DArray, object_location_cb)    
    rate = rospy.Rate(20) # The rate of the loop is no faster than then the readtimeout.
    while not rospy.is_shutdown():
        rate.sleep()

    # Conduct a clean shutdown upon exiting main while loop.    
    takserver.flush()  # flush the xmls the server sends
    rospy.loginfo("Closing TAK Server")
    takserver.close()   
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass    
    
    
    
    


'''


'''


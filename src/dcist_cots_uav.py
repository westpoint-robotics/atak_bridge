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
import rospkg
import tf
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from arl_nav_msgs.msg import GotoRegionActionGoal # Used to publish target location as a goto goal
from nav_msgs.msg import Odometry

from takpak.mkcot import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL

node_name='dcist_cots'
zone='18T'

# Setup ROS node
rospy.init_node(node_name, anonymous=True)

robot_name = rospy.get_param('~name', "warty")
goal_topic="/"+robot_name+"/goto_region/goal"
pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)

# Setup ATAK identity of this node 
my_callsign = rospy.get_param('~callsign', 'default_callsign')
if rospy.has_param('~uid'):
    my_uid = rospy.get_param('~uid')
else:
    my_uid = str(socket.getfqdn()) + "-" + str(uuid.uuid1())[-12:]
my_team_name = rospy.get_param('~team_name', 'Default Team')   
my_team_role = rospy.get_param('~team_role', 'Default Team Role')
tak_ip = rospy.get_param('~tak_ip', '127.0.0.1') 
tak_port = rospy.get_param('~tak_port', '8088')
baselink_frame = rospy.get_param('~baselink_frame', 'base_link') 
global_frame = rospy.get_param('~global_frame', 'utm') 
rospy.loginfo("my_callsign= %s, my_uid= %s, my_team_name= %s, my_uid= %s, my_baselink= %s" %(my_callsign,my_uid,my_team_name,my_team_role,baselink_frame))

tf1_listener = tf.TransformListener()
tf1_listener.waitForTransform(global_frame, baselink_frame, rospy.Time(0), rospy.Duration(5.0))

# Start ATAK client
rospy.loginfo("============ Node: %s is connecting to  TAK Server ===============", node_name)
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

# Setup ROS message to publish
target_msg = Vector3Stamped()

# Setup and run the main while loop
count = 1 # used to keep connection alive

loop_hz = 20
rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the timeout.
while not rospy.is_shutdown():

    # Listen to the server and get message sent
    cotresponse =''
    try: # TODO Use a non-blocking read of the socket
        cotresponse = takserver.readcot(readtimeout=1) # This is a blocking read for 1 second.
        cot_xml = cotresponse[0]
        #rospy.loginfo("COT XML:\n%s" %(cot_xml))
    except:
        rospy.logdebug("Read Cot failed: %s" % (sys.exc_info()[0]))
        #continue        

    # Parse a received message and if it is a move to command publish a move to message
    try:    
        tree = ET.fromstring(cot_xml)
        # Get the UID
        this_uid = tree.get("uid")   
        fiveline = tree.find("./detail/fiveline")
        rospy.loginfo("fiveline:%s" %fiveline)
        target_num = fiveline.attrib['fiveline_target_number']
        # If this is a goto location then publish it as a go to goal.
        
        # Assumes utm is the global frame.
        if ('99999' == target_num):
            lat = tree.find("./point").attrib['lat']
            lon = tree.find("./point").attrib['lon']
            (zone,crnt_utm_e,crnt_utm_n) = LLtoUTM(23, float(lat), float(lon))
            goal_pose_stamped = PoseStamped()
            goal_pose_stamped.header.stamp = rospy.Time.now()  
            rospy.loginfo("target_num:%s" %target_num) 
            goal_pose_stamped.header.frame_id = global_frame   
            rospy.loginfo("lat:%s" %lat)      
            goal_pose_stamped.pose.position.x = crnt_utm_e                    
            goal_pose_stamped.pose.position.y = crnt_utm_n
            rospy.loginfo("lat:%s" %lat)
            msg = goal_pose_stamped   
            pub.publish(msg)           
            rospy.loginfo("----- Recieved ATAK Message from UID: %s, saying move to lat/lon of %s, %s and map location %s, %s" %(this_uid,lat,lon, crnt_utm_e, crnt_utm_n))
    except Exception, e:
        rospy.logwarn("----- Recieved ATAK Message and it is not a move to command -----"+ str(e))

    # ===============================================      
    # Ping the server at 1Hz to keep alive connection 
    if count > 20:
        rospy.logdebug("Keep alive sent -----------------------------------------")
        takserver.send(mkcot.mkcot(cot_ping=True, 
            cot_type="t", 
            cot_how="m-g", 
            cot_callsign=my_callsign, 
            cot_id=my_uid, 
            team_name=my_team_name, 
            team_role=my_team_role)) 
        count = 0 

    # =================================      
    # Get current position in global frame        
    crnt_pose = tf1_listener.lookupTransform(global_frame, baselink_frame, rospy.Time(0))
    (crnt_latitude,crnt_longitude) = UTMtoLL(23, crnt_pose[0][1], crnt_pose[0][0], zone) # 23 is WGS-84.                      
    # Send the current position to the TAK Server      
    takserver.send(mkcot.mkcot(cot_identity="friend", 
        cot_stale = 1, 
        #cot_dimension="land-unit",
        cot_type="a-f-G-M-F-Q",
        #cot_type="a-f-G-U-C", 
        cot_how="m-g", 
        cot_callsign=my_callsign, 
        cot_id=my_uid, 
        team_name=my_team_name, 
        team_role=my_team_role,
        cot_lat=crnt_latitude,
        cot_lon=crnt_longitude )) 
         
    count += 1
    rate.sleep()

# Conduct a clean shutdown upon exiting main while loop.    
takserver.flush()  # flush the xmls the server sends
rospy.loginfo("Closing TAK Server")
takserver.close()    
    


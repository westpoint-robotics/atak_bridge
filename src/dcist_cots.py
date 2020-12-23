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

from takpak.mkcot import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL

zone='18T'

# Setup ROS node
rospy.init_node('dcist_cots', anonymous=True)

robot_name = rospy.get_param('~name', "warty")
goal_topic="/"+robot_name+"/goto_region/goal"
pub = rospy.Publisher(goal_topic, GotoRegionActionGoal, queue_size=10)

tf1_listener = tf.TransformListener()
tf1_listener.waitForTransform("husky/map", "utm", rospy.Time(0), rospy.Duration(5.0))

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
rospy.loginfo("my_callsign=%s, my_uid =%s, my_team_name =%s, my_uid =%s" %(my_callsign,my_uid,my_team_name,my_team_role))

# Start ATAK server
rospy.loginfo("============ Connecting to  TAK Server ===============")
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


def object_cb(data):
    global takserver
    global zone
    for item in data.markers:
        if item.ns == "window": 
            obj_pose_stamped = PoseStamped()
            obj_pose_stamped.header = item.header  
            obj_pose_stamped.header.stamp = rospy.Time.now()                       
            obj_pose_stamped.pose = item.pose
            obj_pose = tf1_listener.transformPose("utm", obj_pose_stamped)
            #print(str(item.header.stamp.secs)+ " " + item.ns + " " + str(item.pose.position.x) + " " + str(item.pose.position.y) + " " + str(item.pose.position.z))
            
            (obj_latitude,obj_longitude) = UTMtoLL(23, obj_pose.pose.position.y, obj_pose.pose.position.x, zone) # 23 is WGS-84.          
            #print(str(item.header.stamp.secs)+ " " + item.ns + " " + str(obj_pose.pose.position.x) + " " + str(obj_pose.pose.position.y) + " " + str(obj_pose.pose.position.z) + " " + str(obj_latitude) + " " + str(obj_longitude))
            takserver.send(mkcot.mkcot(cot_identity="neutral", 
                cot_stale = 1, 
                #cot_dimensionlon="land-unit",
                cot_type="a-f-G-M-F-Q",
                #cot_type="a-f-G-U-C", 
                cot_how="m-g", 
                cot_callsign=item.ns, 
                cot_id="object", 
                team_name="deteczone='18T'tor", 
                team_role="obj detector",
                cot_lat=obj_latitude,
                cot_lon=obj_longitude ))              

    
rospy.Subscriber("/husky/worldmodel_rviz/object_markers", MarkerArray, object_cb)

# Setup Logging TODO Does not work. 
# FIXME the logging system used in takpak does not work with ROS.
# logger = logging.getLogger(f'rosout.{__name__}') # Needed to enable the loggers in takcot modules with ros

# Setup ROS message to publish
target_msg = Vector3Stamped()

# Setup and run the main while loop
count = 1 # used to keep connection alive

loop_hz = 20
rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the readtimeout.
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
        target_num = fiveline.attrib['fiveline_target_number']
        if ('99999' == target_num): # TODO change to publish a go to goal message
            lat = tree.find("./point").attrib['lat']
            lon = tree.find("./point").attrib['lon']
            (zone,crnt_utm_e,crnt_utm_n) = LLtoUTM(23, float(lat), float(lon))
            crnt_pose_stamped = PoseStamped()
            crnt_pose_stamped.header.stamp = rospy.Time.now()   
            crnt_pose_stamped.header.frame_id = 'utm'         
            crnt_pose_stamped.pose.position.x = crnt_utm_e                    
            crnt_pose_stamped.pose.position.y = crnt_utm_n 
            crnt_pose = tf1_listener.transformPose("husky/map", crnt_pose_stamped)
            tgt_pos_x = crnt_pose.pose.position.x
            tgt_pos_y = crnt_pose.pose.position.y            
            msg = GotoRegionActionGoal()
            msg.header.stamp = rospy.Time.now()
            msg.goal_id.stamp = rospy.Time.now()
            msg.goal_id.id = "ATAK GOTO"
            msg.goal.region_center.header.stamp = crnt_pose_stamped.header.stamp
            msg.goal.region_center.header.frame_id = "husky/map"
            msg.goal.region_center.pose.position.x = tgt_pos_x
            msg.goal.region_center.pose.position.y = tgt_pos_y
            msg.goal.region_center.pose.orientation.z = 0.0985357937255
            msg.goal.region_center.pose.orientation.w = 0.995133507302
            msg.goal.radius = 3.75
            msg.goal.angle_threshold = 3.108    
            pub.publish(msg)           
            rospy.loginfo("----- Recieved ATAK Message from UID: %s, saying move to lat/lon of %s, %s and map location %s, %s" %(this_uid,lat,lon, tgt_pos_x, tgt_pos_y))
    except Exception, e:
        rospy.logdebug("----- Recieved ATAK Message and it is not a move to command -----"+ str(e))

    count += 1
    # TODO Determine if this needed, if we are sending pose updates periodically we may not need this.
    if count > 20: # do these actions once second.
        rospy.logdebug("Keep alive sent -----------------------------------------")
        takserver.send(mkcot.mkcot(cot_ping=True, 
            cot_type="t", 
            cot_how="m-g", 
            cot_callsign=my_callsign, 
            cot_id=my_uid, 
            team_name=my_team_name, 
            team_role=my_team_role)) 
        count = 0  
        
        
    # Get currnet position in Map frame        
    crnt_pose = tf1_listener.lookupTransform('utm', 'husky/base_link', rospy.Time(0))
    #([0.17887300879040938, -0.048116981667806785, 0.014201157337402178], [-0.002688032953270503, 0.0011190697035904009, -0.11106424779955845, 0.9938089630419717])
    (crnt_latitude,crnt_longitude) = UTMtoLL(23, crnt_pose[0][1], crnt_pose[0][0], zone) # 23 is WGS-84.  
    
                    
    # ============================  
    # Send the current position to the TAK Server    
    # TODO Send this at appropriate rate. Currently it is tied to the readtimeout.    
    #takserver.flush()  # flush the xmls the server sends    
    takserver.send(mkcot.mkcot(cot_identity="friend", 
        cot_stale = 1, 
        #cot_dimension="land-unit",
        #cot_type="a-f-G-M-F-Q",
        cot_type="a-f-G-U-C", 
        cot_how="m-g", 
        cot_callsign=my_callsign, 
        cot_id=my_uid, 
        team_name=my_team_name, 
        team_role=my_team_role,
        cot_lat=crnt_latitude,
        cot_lon=crnt_longitude ))  
    rate.sleep()

# Conduct a clean shutdown upon exiting main while loop.    
takserver.flush()  # flush the xmls the server sends
rospy.loginfo("Closing TAK Server")
takserver.close()    
    


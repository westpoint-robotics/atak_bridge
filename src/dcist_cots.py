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
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix

from takpak.mkcot import mkcot
from takpak.takcot import takcot


# Setup ROS node
pub = rospy.Publisher('atak_tgt', Vector3Stamped, queue_size=10)
rospy.init_node('atak_targets', anonymous=True)
rospack = rospkg.RosPack()
pack_dir = rospack.get_path('rospy_tutorials')

# Setup ROS callback to keep an updated position of the robot
current_location = NavSatFix()
current_location.latitude = 41.393850
current_location.latitude = -73.953674
def update_location_cb(data):
    global current_location
    current_location = data

# Setup Logging TODO Does not work.
# logger = logging.getLogger(f'rosout.{__name__}') # Needed to enable the loggers in takcot modules with ros

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

# Setup ROS message to publish
target_msg = Vector3Stamped()

# Setup and run the main while loop
count = 1 # used to keep connection alive
pub = rospy.Publisher('atak_tgt', Vector3Stamped, queue_size=10)
rospy.init_node('atak_targets', anonymous=True)
rospy.Subscriber("atak_fix", NavSatFix, update_location_cb)
rate = rospy.Rate(20) # 10hz
while not rospy.is_shutdown():

    # Listen to the server and get message sent
    cotresponse =''
    try:
        cotresponse = takserver.readcot(readtimeout=1)
        cot_xml = cotresponse[0]
        #rospy.loginfo("%s" %(cot_xml))
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
        if ('99999' == target_num):
            lat = tree.find("./point").attrib['lat']
            lon = tree.find("./point").attrib['lon']
            rospy.loginfo("----- Recieved ATAK Message from UID: %s, saying move to %s, %s" %(this_uid,lat,lon))
            target_msg.header.stamp = rospy.Time.now()
            target_msg.vector.x = float(lat)
            target_msg.vector.y = float(lon)
            pub.publish(target_msg)            
    except:
        rospy.logdebug("----- Recieved ATAK Message and it is not a move to command -----")


    # Send a keep-alive once a second    
    count += 1
    if count > 20:
        rospy.logdebug("Keep alive sent -----------------------------------------")
        takserver.send(mkcot.mkcot(cot_ping=True, 
            cot_type="t", 
            cot_how="m-g", 
            cot_callsign=my_callsign, 
            cot_id=my_uid, 
            team_name=my_team_name, 
            team_role=my_team_role)) 
        count = 1  
        
         
    # ============================  
    # Send the current position to the TAK Server        
    takserver.flush()  # flush the xmls the server sends
    takserver.send(mkcot.mkcot(cot_identity="friend", 
        cot_stale = 1, 
        #cot_dimension="land-unit",
        cot_type="a-f-G-M-F-Q", 
        cot_how="m-g", 
        cot_callsign=my_callsign, 
        cot_id=my_uid, 
        team_name=my_team_name, 
        team_role=my_team_role,
        cot_lat=current_location.latitude,
        cot_lon=current_location.longitude ))        

    rate.sleep()


# Conduct a clean shutdown upon exiting main while loop.    
takserver.flush()  # flush the xmls the server sends
rospy.loginfo("Closing TAK Server")
takserver.close()    
    


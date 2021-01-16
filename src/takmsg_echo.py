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

from takpak.mkcot import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL

node_name='atak_echo'
zone='18T'

# Setup ROS node
rospy.init_node(node_name, anonymous=True)

# Setup ATAK identity of this node 
my_callsign = rospy.get_param('~callsign', 'default_callsign')
if rospy.has_param('~uid'):
    my_uid = rospy.get_param('~uid')
else:
    my_uid = str(socket.getfqdn()) + "-" + str(uuid.uuid1())[-12:] + '_echo'
my_team_name = rospy.get_param('~team_name', 'Default Team')   
my_team_role = rospy.get_param('~team_role', 'Default Team Role')
tak_ip = rospy.get_param('~tak_ip', '10.13.0.10') 
tak_port = rospy.get_param('~tak_port', '8088') 
rospy.loginfo("my_callsign=%s, my_uid =%s, my_team_name =%s, my_uid =%s" %(my_callsign,my_uid,my_team_name,my_team_role))

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
    
# Setup and run the main while loop
count = 0 # used to keep connection alive
loop_hz = 5
rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the readtimeout.
while not rospy.is_shutdown():
    # Listen to the server and get message sent
    cotresponse =''
    try: # TODO Use a non-blocking read of the socket
        cotresponse = takserver.readcot(readtimeout=1) # This is a blocking read for 1 second.
        cot_xml = cotresponse[0]
        rospy.loginfo("COT XML:\n%s" %(cot_xml))
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
        rospy.info("TAKMSG:-----\n\n%s"+ str(tree))
        if ('99999' == target_num): # TODO change to publish a go to goal message
            lat = tree.find("./point").attrib['lat']
            lon = tree.find("./point").attrib['lon']        
            rospy.loginfo("----- Recieved ATAK Message from UID: %s, saying move to lat/lon of %s, %s" %(this_uid,lat,lon))
    except Exception, e:
        rospy.logdebug("----- Recieved ATAK Message and it is not a move to command -----"+ str(e))

    count += 1
    if count > 20: # ping server once a second to keep alive.
        rospy.logdebug("Keep alive sent -----------------------------------------")
        takserver.send(mkcot.mkcot(cot_ping=True, 
            cot_type="t", 
            cot_how="m-g", 
            cot_callsign=my_callsign, 
            cot_id=my_uid, 
            team_name=my_team_name, 
            team_role=my_team_role)) 
        count = 0  
    rate.sleep()

# Conduct a clean shutdown upon exiting main while loop.    
takserver.flush()  # flush the xmls the server sends
rospy.loginfo("Closing TAK Server")
takserver.close()    
    


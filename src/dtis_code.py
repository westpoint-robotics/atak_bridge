#!/usr/bin/env python
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

'''
  <node type="static_transform_publisher" pkg="tf2_ros" name="dummy2_utm_frame" args="587361 4582574 0 0 0 1.571 utm $(arg name)/map" />

rosrun tf2_ros static_transform_publisher 587361 4582574 0 0 0 1.571 utm map
'''

import os
import sys
import time
import uuid
import socket
import numpy as np

import json
import xml.etree.ElementTree as ET
from  xml.dom.minidom import parseString
import logging
from takpak.mkcot import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL

#TODO Investigate handling of TAK messages arriving faster than being processed by this code.
#TODO Find a sufficient answer to send duplicate targets. Seeing 2 tgts and reporting one, and the other way around.

class AtakBridge:
    """A class used to communication between an ATAK and robots"""
    
    def __init__(self):
        self.robot_name          = "DTIS"
        self.my_team_name        = 'Cyan' #Use one from ATAK which are colors  
        self.my_team_role        = 'Team Member' # Use one from ATAK
        self.tak_ip              = '10.13.0.10' 
        self.tak_port            = '8088'
        self.baselink_frame      = 'base_link' 
        self.global_frame        = 'utm'
        self.my_callsign         = 'default_callsign'
        self.my_uid              = self.set_uid()
        self.zone='18T'
        self.takmsg_tree = ''
        print("Started ATAK Bridge with the following:\n\t\tCallsign: %s\n\t\tUID: %s\n\t\tTeam name: %s\n\t\tGlobal Frame: %s"
                    %(self.my_callsign,self.my_uid,self.my_team_name,self.global_frame))
        self.takserver = takcot() #TODO add a timeout and exit condition                    
        
                             
        
    def set_uid(self):
        """Set the UID using either a rosparam or the system uuid"""
        uid = "ugv_"+str(socket.getfqdn()) + "-" + str(uuid.uuid1())[-12:]  
        return uid  
            
        
    def distancePoses(self, pose1, pose2):
        """Determine the distance between two ROS pose data types"""                
        p1 = np.array([pose1.position.x,pose1.position.y,pose1.position.z])
        p2 = np.array([pose2.position.x,pose2.position.y,pose2.position.z])
        squared_dist = np.sum((p1-p2)**2, axis=0)
        dist = np.sqrt(squared_dist)
        return dist
        
    def takserver_start(self):
        print("============ Connecting to  TAK Server at %s:%s ===============" %(self.tak_ip,self.tak_port))
        print("==== If the code appears to freeze at this point, then it is likely the server is not reachable  =====")    
        try:
            tasksock = self.takserver.open(self.tak_ip, self.tak_port)
            self.takserver.flush()          
        except:
            e = sys.exc_info()[0]
            print("Failed to connect to the TAK Server", e)
            exit()

        # Send a ping and check for answer
        connect_xml = (mkcot.mkcot(cot_type="t", cot_how="h-g-i-g-o", cot_callsign=self.my_callsign, 
                       cot_id=self.my_uid, team_name=self.my_team_name, team_role=self.my_team_role)).decode('utf-8')
        my_xml = parseString(str(connect_xml.replace("\n",""))).toprettyxml()
        tree = ET.fromstring(my_xml)
        xml_callsign = tree.find("./detail/contact").attrib['callsign']
        print("SUCCESSFUL Conection with the server. The server believes my callsign is: %s" %(xml_callsign))
        
    def takserver_shutdown(self):
        # Conduct a clean shutdown upon exiting main while loop.    
        self.takserver.flush()  # flush the xmls the server sends
        print("Closing TAK Server")
        self.takserver.close()    
        
    def takserver_read(self):
        # Listen to the server and get message sent
        cotresponse =''
        try: # TODO Use a non-blocking read of the socket
            cotresponse = self.takserver.readcot(readtimeout=1) # This is a blocking read for 1 second.
            cot_xml = cotresponse[0]
            print("COT XML:\n%s" %(cot_xml))
            if (len(cot_xml)>1):
                self.takmsg_tree = ET.fromstring(cot_xml)
        except:
            print("Read Cot failed: %s" % (sys.exc_info()[0]))
                
    def robot_pose_to_tak(self):
        # Get current position in global frame        
        #crnt_pose = self.tf1_listener.lookupTransform('utm', self.baselink_frame, rospy.Time(0)) #get GPS lat and long and feed to next line
        #(crnt_latitude,crnt_longitude) = UTMtoLL(23, crnt_pose[0][1], crnt_pose[0][0], self.zone) # 23 is WGS-84.                      
        (crnt_latitude,crnt_longitude) = (-73,48)
        
        # Send the current position to the TAK Server  
        #print("latlong: %.7f,%.7f baselinkg is: %s"%(crnt_latitude,crnt_longitude, self.baselink_frame))    
        my_cot = mkcot.mkcot(cot_identity="friend", 
            cot_stale = 1, 
            cot_type="a-f-G-M-F-Q",
            cot_how="m-g", 
            cot_callsign=self.my_callsign, 
            cot_id=self.my_uid, 
            team_name=self.my_team_name, 
            team_role=self.my_team_role,
            cot_lat=crnt_latitude,
            cot_lon=crnt_longitude )  
        self.takserver.send( my_cot)   
        #print(my_cot) 
        #copy my_cot



if __name__ == '__main__':
    bridge = AtakBridge()
    bridge.takserver_start()
    while True:
        bridge.robot_pose_to_tak()
        #insert a python sleep here
        time.sleep(1)
    bridge.takserver_shutdown()


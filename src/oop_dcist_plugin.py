#!/usr/bin/env python

from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

'''
  <node type="static_transform_publisher" pkg="tf2_ros" name="dummy2_utm_frame" args="587361 4582574 0 0 0 1.571 utm $(arg name)/map" />

rosrun tf2_ros static_transform_publisher 587361 4582574 0 0 0 1.571 utm husky/map
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

import rospy
import rospkg
import tf

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

#from vision_msgs.msg import Detection2DArray #TODO this message is what is used in mavplatform, does not exist in phoenix

from takpak.mkcot import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL

#TODO Investigate handling of TAK messages arriving faster than being processed by this code.
#TODO Find a sufficient answer to send duplicate targets. Seeing 2 tgts and reporting one, and the other way around.

class AtakBridge:
    """A class used to communication between an ATAK and robots"""
    
    def __init__(self):
        self.robot_name          = rospy.get_param('~name', "husky")
        self.my_team_name        = rospy.get_param('~team_name', 'Cyan') #Use one from ATAK which are colors  
        self.my_team_role        = rospy.get_param('~team_role', 'Team Member') # Use one from ATAK
        self.tak_ip              = rospy.get_param('~tak_ip', '127.0.0.1') 
        self.tak_port            = rospy.get_param('~tak_port', '8088')
        self.baselink_frame      = rospy.get_param('~baselink_frame', 'base_link') 
        self.global_frame        = rospy.get_param('~global_frame', 'utm')
        self.my_callsign         = rospy.get_param('~callsign', 'default_callsign')
        self.my_uid              = self.set_uid()
        self.zone='18T'
        self.takmsg_tree = ''
        self.target_list = ["people"]#["car", "Vehicle"]
        self.msg_id_num=0
        self.marker_topic="/"+self.robot_name+"/atak/goal"
        self.vis_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)
        #self.goal_topic="/"+self.robot_name+"/nav_goal/2d"        
        #self.uav_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        self.goal_topic="/"+self.robot_name+"/goto_region/goal"
        
        # rospy.Subscriber("/husky/worldmodel_rviz/object_markers", MarkerArray, self.grnd_object_cb)
        #rospy.Subscriber("/uav1/detection_localization/detections/out/local", Detection2DArray, self.object_location_cb)
        rospy.loginfo("Started ATAK Bridge with the following:\n\t\tCallsign: %s\n\t\tUID: %s\n\t\tTeam name: %s\n\t\tGlobal Frame: %s"
                    %(self.my_callsign,self.my_uid,self.my_team_name,self.global_frame))
        self.takserver = takcot() #TODO add a timeout and exit condition                    
        
    def set_uid(self):
        """Set the UID using either a rosparam or the system uuid"""
        if rospy.has_param('~uid'):
            uid = rospy.get_param('~uid')
        else:
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
        rospy.loginfo("============ Connecting to  TAK Server at %s:%s ===============" %(self.tak_ip,self.tak_port))
        rospy.loginfo("==== If the code appears to freeze at this point, then it is likely the server is not reachable  =====")    
        try:
            tasksock = self.takserver.open(self.tak_ip, self.tak_port)
            self.takserver.flush()          
        except:
            rospy.logerr("Failed to connect to the TAK Server")
            exit()

        # Send a ping and check for answer
        connect_xml = (mkcot.mkcot(cot_type="t", cot_how="h-g-i-g-o", cot_callsign=self.my_callsign, 
                       cot_id=self.my_uid, team_name=self.my_team_name, team_role=self.my_team_role)).decode('utf-8')
        my_xml = parseString(str(connect_xml.replace("\n",""))).toprettyxml()
        tree = ET.fromstring(my_xml)
        xml_callsign = tree.find("./detail/contact").attrib['callsign']
        rospy.loginfo("SUCCESSFUL Conection with the server. The server believes my callsign is: %s" %(xml_callsign))
        
    def takserver_shutdown(self):
        # Conduct a clean shutdown upon exiting main while loop.    
        self.takserver.flush()  # flush the xmls the server sends
        rospy.loginfo("Closing TAK Server")
        self.takserver.close()    
        
    def takserver_read(self):
        # Listen to the server and get message sent
        cotresponse =''
        #rospy.loginfo("INSIDE takserver_read")
        try: # TODO Use a non-blocking read of the socket
            cotresponse = self.takserver.readcot(readtimeout=1) # This is a blocking read for 1 second.
            cot_xml = cotresponse[0]
            if (len(cot_xml)>1):
                #rospy.loginfo("COT XML:\n%s\n" %(cot_xml))
                #self.takmsg_tree = ET.fromstring(cot_xml)
                self.takmsg_tree = ET.ElementTree(ET.fromstring(cot_xml))
                msg_data = self.parse_takmsg_plugin()
                if (len(msg_data) > 2):
                    # send goto message to robot
                    rospy.loginfo("\n============>  %s, %s, %s, %s\n" %(msg_data[0],msg_data[1],msg_data[2],msg_data[3])) 
        except:
            rospy.logdebug("Read Cot failed: %s" % (sys.exc_info()[0]))

    def parse_takmsg_plugin(self):
        try:                
            msg_tree = self.takmsg_tree.getroot()            
            if not(msg_tree in (-1, None)):
                msg_uid = msg_tree.attrib['uid']
                if ('husky_1_goto' == msg_uid):
                    lat = self.takmsg_tree.find("./point").attrib['lat']
                    lon = self.takmsg_tree.find("./point").attrib['lon']
                    detail = self.takmsg_tree.find("./detail").find("./remarks").text.split(',')
                    desired_orientation = detail[2]
                    desired_altitude = detail[3]
                    (zone,crnt_utm_e,crnt_utm_n) = LLtoUTM(23, float(lat), float(lon))
                    #rospy.loginfo("\n\n----- Recieved ATAK Message from UID: %s, saying move to lat/lon of %s, %s heading: %s altitude: %s\n\n" %(msg_uid,lat,lon,desired_orientation,desired_altitude)) 
                    return (lat,lon,desired_orientation,desired_altitude)
            return 'na' # Not a message for the robot
        except Exception, e:
            rospy.logwarn("\n\n----- Recieved ATAK Message and have an error of: "+ str(e) + '\n\n') 
            return 'na' # Cant parse the ATAK message

    def robot_pose_to_tak(self):
        # Get current position in global frame
        (crnt_latitude,crnt_longitude) = (41.39081900138365, -73.9531831888787)
        my_cot = mkcot.mkcot(cot_identity="friend", 
            cot_stale = 0.1, 
            cot_type="a-f-G-M-F-Q",
            cot_how="m-g", 
            cot_callsign=self.my_callsign, 
            cot_id=self.my_uid, 
            team_name=self.my_team_name, 
            team_role=self.my_team_role,
            cot_lat=crnt_latitude,
            cot_lon=crnt_longitude )  
        self.takserver.send( my_cot)   


if __name__ == '__main__':
    try:
        rospy.init_node("atak_bridge")
        bridge = AtakBridge()
        bridge.takserver_start()
        loop_hz = 10
        rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the timeout.
        while not rospy.is_shutdown():
            bridge.takserver_read()
            #bridge.parse_takmsg_grnd()
            rate.sleep()
        
        bridge.takserver_shutdown()
    except rospy.ROSInterruptException:
        pass        
        
        
'''
Example goto cot message

<?xml version="1.0" encoding="UTF-8"?>
<event version="2.0" uid="husky_1_goto" type="b-m-p-w-GOTO" time="2021-09-06T17:36:18Z" start="2021-09-06T17:36:17.945Z" stale="2021-09-06T17:41:17.945Z" how="h-g-i-g-o">
    <point lat="41.3864501" lon="-73.9580827" hae="0.0" ce="9999999.0" le="9999999.0"/>
    <detail>
        <status readiness="true"/>
        <archive/>
        <contact callsign="husky_1_goto"/>
        <remarks>RRC,husky_1,NA,NA</remarks>
        <link uid="ANDROID-a4d683ba2030aebd" production_time="2021-09-06T17:29:28.965Z" type="a-f-G-U-C" parent_callsign="LYNX" relation="p-p"/>
        <color argb="-1"/>
        <archive/>
        <_flow-tags_ TAK-Server-c5a19b2b235f433aaa1a8e10d67165b4="2021-09-06T17:36:18Z"/>
    </detail>
</event>


'''        


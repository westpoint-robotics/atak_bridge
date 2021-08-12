#!/usr/bin/env python
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

import os
import sys
import time
import uuid
import socket
import numpy as np
import math

import json
import xml.etree.ElementTree as ET
from  xml.dom.minidom import parseString
import logging

import rospy
import rospkg
import tf

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3Stamped
#from arl_nav_msgs.msg import GotoRegionActionGoal # Used to publish target location as a goto goal
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray

from takpak.mkcot import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL


from scipy.spatial.transform import Rotation

#TODO Investigate handling of TAK messages arriving faster than being processed by this code.
#TODO Find a sufficient answer to send duplicate targets. Seeing 2 tgts and reporting one, and the other way around.

class AtakBridge:
    """A class used to communication between an ATAK and robots"""
    
    def __init__(self):
        self.robot_name          = rospy.get_param('~name', "warty")
        self.my_team_name        = rospy.get_param('~team_name', 'Default Team')   
        self.my_team_role        = rospy.get_param('~team_role', 'Default Team Role')
        self.tak_ip              = rospy.get_param('~tak_ip', '127.0.0.1') 
        self.tak_port            = rospy.get_param('~tak_port', '8088')
        self.baselink_frame      = rospy.get_param('~baselink_frame', 'base_link') 
        self.global_frame        = rospy.get_param('~global_frame', 'utm')
        self.my_callsign         = rospy.get_param('~callsign', 'default_callsign')
        self.my_uid              = self.set_uid()
        
        #TODO Receive gps data from launch file
        self.zone                = rospy.get_param('~utm_zone', '18T')   # USMA
        # West Point
        self.start_lat   = rospy.get_param('~start_lat', '41.39058')
        self.start_long   = rospy.get_param('~start_long', '-73.953230')
        # ARL sim start position
#        self.start_arl_lat   = 39.476528 
#        self.start_arl_lon   = -76.083900
        
        self.takmsg_tree = ''
        self.target_list = ["person", "car", "boat", "8"]
        self.tf1_listener = tf.TransformListener()
        self.tf1_listener.waitForTransform(self.global_frame, self.baselink_frame, rospy.Time(0), rospy.Duration(35.0))

        self.goal_topic ="/"+self.robot_name+"/nav_goal/2d"
        self.uav_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        
        self.topic_target = "/"+self.robot_name+"/detection_localization/out/detections/local"
        rospy.Subscriber(self.topic_target, Detection2DArray, self.object_location_cb)
        rospy.loginfo("Started ATAK Bridge with the following:\n\t\tCallsign: %s\n\t\tUID: %s\n\t\tTeam name: %s"
                    %(self.my_callsign,self.my_uid,self.my_team_name))
        self.takserver = takcot() #TODO add a timeout and exit condition                    
        
    def object_location_cb(self, data):        
        for detection in data.detections: 
            for result in detection.results:
#                rospy.loginfo("%s" %result.id)
                if str(result.id) in self.target_list:
#                    rospy.loginfo("%s" %result.id)
                    obj_pose_stamped = PoseStamped()
                    obj_pose_stamped.header = detection.header 
                    obj_pose_stamped.pose = result.pose.pose
                    obj_pose_utm = self.tf1_listener.transformPose("utm", obj_pose_stamped)             
                    
                    # *****
                    (zone,start_utm_x,start_utm_y) = LLtoUTM(23, self.start_lat, self.start_lon)
#                    (zone,start_utm_arl_x,start_utm_arl_y) = LLtoUTM(23, self.start_arl_lat, self.start_arl_lon)       
                    obj_utm_x = obj_pose_utm.pose.position.x - start_utm_arl_x + start_utm_x
                    obj_utm_y = obj_pose_utm.pose.position.y - start_utm_arl_y + start_utm_y 

                    
                    (obj_latitude,obj_longitude) = UTMtoLL(23, obj_utm_y, obj_utm_x, self.zone) # 23 is WGS-84.
                                                 
                    self.takserver.send(mkcot.mkcot(cot_identity="neutral", 
                        cot_stale = 1, 
                        cot_type="a-f-G-M-F-Q",
                        cot_how="m-g", 
                        cot_callsign=str(result.id), 
                        cot_id="object", 
                        team_name="detector", 
                        team_role="obj detector",
                        cot_lat=obj_latitude,
                        cot_lon=obj_longitude ))                    

        
    def set_uid(self):
        """Set the UID using either a rosparam or the system uuid"""
        if rospy.has_param('~uid'):
            uid = rospy.get_param('~uid')
        else:
            uid = str(socket.getfqdn()) + "-" + str(uuid.uuid1())[-12:]  
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
        try: # TODO Use a non-blocking read of the socket
            cotresponse = self.takserver.readcot(readtimeout=1) # This is a blocking read for 1 second.
            cot_xml = cotresponse[0]
            rospy.logdebug("COT XML:\n%s" %(cot_xml))
            if (len(cot_xml)>1):
                self.takmsg_tree = ET.fromstring(cot_xml)
        except:
            rospy.logdebog("Read Cot failed: %s" % (sys.exc_info()[0]))
            
    def parse_takmsg(self):
        fiveline = self.takmsg_tree.find("./detail/fiveline")
        #rospy.loginfo("fiveline:%s" %fiveline)
        if not(fiveline in (-1, None)):
            tgt_num = fiveline.attrib['fiveline_target_number']
            # If this is a goto location then publish it as a go to goal.
            # Assumes utm is the global frame.
            rospy.loginfo('------------------')
            rospy.loginfo(tgt_num)
            
            # Input: 'U99' + 'desired heading angle (3 digits)'                        
            if (tgt_num[0:3] == 'U99'):
                try:
                    this_uid = self.takmsg_tree.get("uid")
                    tgt_lat = self.takmsg_tree.find("./point").attrib['lat']
                    tgt_long = self.takmsg_tree.find("./point").attrib['lon']
                except Exception as e:
                    rospy.logwarn("----- Recieved ATAK Message and it is not a move to command -----"+ str(e))


                (tgt_zone, tgt_utm_e, tgt_utm_n) = LLtoUTM(23, float(tgt_lat), float(tgt_long))
                
                (start_zone, start_utm_x, start_utm_y) = LLtoUTM(23, float(self.start_lat), float(self.start_long))

                #
                tgt_utm_x = tgt_utm_e - start_utm_x
                tgt_utm_y = tgt_utm_n - start_utm_y
                               
                rospy.loginfo("%f, %f" %(tgt_utm_x, tgt_utm_y))
                
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.header.stamp = rospy.Time.now()
                goal_pose_stamped.header.frame_id = 'utm'
                goal_pose_stamped.pose.position.x = tgt_utm_x
                goal_pose_stamped.pose.position.y = tgt_utm_y
                
                # utm >> odom
                msg = self.tf1_listener.transformPose(self.global_frame, goal_pose_stamped)
                
                # Target Assignment              
                msg.pose.position.x = tgt_utm_x
                msg.pose.position.y = tgt_utm_y
                #TODO: allow to change alt by using tgt_num
                msg.pose.position.z = 2.0
                               
                #caluclate current position
                crnt_pose = self.tf1_listener.lookupTransform('utm', self.baselink_frame, rospy.Time(0))
                crnt_utm_x = crnt_pose[0][0]
                crnt_utm_y = crnt_pose[0][1]
 
                
                # Three digits after 'U99' is used to control the heading angle
                # E = 000, W = 180, S = 270, N = 90
                if (tgt_num[0:3] == 'U99') and (int(tgt_num[3:6]) < 360):
#                    rospy.loginfo('+++TURNING+++')
                    rot = Rotation.from_euler('xyz', [0, 0, int(tgt_num[3:6])], degrees=True)
                    rot_quat = rot.as_quat()
#                    rospy.loginfo('++++++++++ yaw: %f , z: %f, w: %f' %(yaw, rot_quat[2], rot_quat[3]))
                    msg.pose.position.x = crnt_utm_x - start_utm_x
                    msg.pose.position.y = crnt_utm_y - start_utm_y
                    msg.pose.orientation.z = rot_quat[2]
                    msg.pose.orientation.w = rot_quat[3]

                else:                
                    #calculate heading angle
                    x_yaw = tgt_utm_e - crnt_utm_x
                    y_yaw = tgt_utm_n - crnt_utm_y
                    yaw = math.atan2(y_yaw,x_yaw)/math.pi*180
                    rot = Rotation.from_euler('xyz', [0, 0, yaw], degrees=True)
                    rot_quat = rot.as_quat()
#                    rospy.loginfo('++++++++++ yaw: %f , z: %f, w: %f' %(yaw, rot_quat[2], rot_quat[3]))
                
                    msg.pose.orientation.z = rot_quat[2]
                    msg.pose.orientation.w = rot_quat[3]
               
#                rospy.loginfo(msg)                
                                
                self.uav_pub.publish(msg)
                rospy.loginfo("----- !!(SIMULATION)!! Recieved ATAK Message from UID: %s, saying move to lat/long of %s, %s and map location %s, %s" %(this_uid, tgt_lat,tgt_long, tgt_utm_x,tgt_utm_y))


    def robot_pose_to_tak(self):
        # Get current position in global frame        
        crnt_pose = self.tf1_listener.lookupTransform('utm', self.baselink_frame, rospy.Time(0))
        
        crnt_utm_x = crnt_pose[0][0]
        crnt_utm_y = crnt_pose[0][1]
        
        (crnt_lat,crnt_long) = UTMtoLL(23, crnt_utm_y, crnt_utm_x, self.zone) # 23 is WGS-84.
        
        # Send the current position to the TAK Server  
        #rospy.loginfo("latlong: %.7f,%.7f baselinkg is: %s"%(crnt_latitude,crnt_longitude, self.baselink_frame))    
        self.takserver.send(mkcot.mkcot(cot_identity="friend", 
            cot_stale    = 1,
            cot_type     = "a-f-G-M-F-Q",
            cot_how      = "m-g",
            cot_callsign = self.my_callsign, 
            cot_id       = self.my_uid, 
            team_name    = self.my_team_name, 
            team_role    = self.my_team_role,
            cot_lat      = crnt_lat,
            cot_lon      = crnt_long ))     
            
        
if __name__ == '__main__':
    try:
        rospy.init_node("atak_bridge")
        bridge = AtakBridge()
        bridge.takserver_start()
        loop_hz = 10
        rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the timeout.
        while not rospy.is_shutdown():
            bridge.takserver_read()
            bridge.parse_takmsg()
            bridge.robot_pose_to_tak()
        
        
            rate.sleep()
        
        
        bridge.takserver_shutdown()
    except rospy.ROSInterruptException:
        pass        

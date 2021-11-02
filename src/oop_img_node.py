#!/usr/bin/env python

# oop_img_test.py
# Jinho Kim (jinho.kim@westpoint.edu)
# Nov. 2021

# Image display function added

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
from nav_msgs.msg import Path

from sensor_msgs.msg import Image
import base64
from cv_bridge import CvBridge, CvBridgeError
import cv2

#### mkcot_beta used!!! ####
from takpak.mkcot_beta import mkcot
from takpak.takcot import takcot

from LatLongUTMconversion import LLtoUTM, UTMtoLL


from scipy.spatial.transform import Rotation


#TODO Investigate handling of TAK messages arriving faster than being processed by this code.
#TODO Find a sufficient answer to send duplicate targets. Seeing 2 tgts and reporting one, and the other way around.

class AtakBridge:
    """A class used to communication between an ATAK and robots"""
    
    def __init__(self):
        self.robot_name          = rospy.get_param('~name', "warty")
        self.my_team_name        = rospy.get_param('~team_name', 'Cyan')   
        self.my_team_role        = rospy.get_param('~team_role', 'Team Member')
        self.tak_ip              = rospy.get_param('~tak_ip', '127.0.0.1') 
        self.tak_port            = rospy.get_param('~tak_port', '8088')
        self.baselink_frame      = rospy.get_param('~baselink_frame', 'base_link') 
        self.map_frame           = rospy.get_param('~map_frame', 'odom')
        self.my_callsign         = rospy.get_param('~callsign', 'default_callsign')
        self.my_uid              = self.set_uid()
        
        #TODO Receive gps data from launch file
        self.zone                = rospy.get_param('~utm_zone', '18T')   # USMA
        # West Point
        self.start_lat           = rospy.get_param('~start_lat', '41.39058')
        self.start_lon           = rospy.get_param('~start_lon', '-73.953230')
        self.tgt_alt             = rospy.get_param('~tgt_alt', '1.5')
        
        self.takmsg_tree         = ''
        self.target_list         = ["person", "0", "truck", "7", "boat", "8"]
        self.tf1_listener        = tf.TransformListener()
        self.tf1_listener.waitForTransform(self.map_frame, self.baselink_frame, rospy.Time(0), rospy.Duration(35.0))

        # Detection
        self.topic_target        = "/"+self.robot_name+"/detection_localization/out/detections/local"
        rospy.Subscriber(self.topic_target, Detection2DArray, self.object_location_cb)

        # Image
        self.img_bridge          = CvBridge()
        self.topic_img           = "/"+self.robot_name+"/tflite_ros/detections_image"
        rospy.Subscriber(self.topic_img, Image, self.image_cb)
        self.img_h               = ''
        self.img_w               = ''
        self.encoded_img         = ''
                
        # Goto mission : wp
        self.goal_topic          ="/"+self.robot_name+"/nav_goal/2d"
        self.goal_pub            = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        
        # Goto mission : path
        self.path_topic          ="/"+self.robot_name+"/nav_goal/path"
        self.path_pub            = rospy.Publisher(self.path_topic, Path, queue_size=10)

        
        rospy.loginfo("Started ATAK Bridge with the following:\n\t\tCallsign: %s\n\t\tUID: %s\n\t\tTeam name: %s"
                    %(self.my_callsign,self.my_uid,self.my_team_name))
        self.takserver           = takcot() #TODO add a timeout and exit condition                    

    def object_location_cb(self, data):
        for detection in data.detections: 
            for result in detection.results:
#                rospy.loginfo("%s" %result.id)
                if str(result.id) in self.target_list:
                    rospy.loginfo('========== Object Detected ==========')
                    obj_pose_stamped = PoseStamped()
                    obj_pose_stamped.header = detection.header 
                    obj_pose_stamped.pose = result.pose.pose
                    obj_pose_stamped.header.frame_id = self.map_frame
                    obj_pose_utm = self.tf1_listener.transformPose("utm", obj_pose_stamped)
                    
                    # *****
                    obj_utm_x = obj_pose_utm.pose.position.x
                    obj_utm_y = obj_pose_utm.pose.position.y
                    (obj_lat, obj_lon) = UTMtoLL(23, obj_utm_y, obj_utm_x, self.zone) # 23 is WGS-84.
                    
                    # Publish position CoT message
#                    self.takserver.send(mkcot.mkcot(cot_identity = "neutral",
#                                                    cot_stale     = 1,
#                                                    cot_type      = "a-f-G-M-F-Q",
#                                                    cot_how       = "m-g",
#                                                    cot_callsign  = str(result.id),
#                                                    cot_id        = "object",
#                                                    team_name     = "detector",
#                                                    team_role     = "obj detector",
#                                                    cot_lat       = obj_lat,
#                                                    cot_lon       = obj_lon ))

                    # Publish img CoT message
                    self.takserver.send(mkcot.mkcot(cot_identity = "neutral",
                                                    cot_stale    = 1,
                                                    cot_type     = "b-i-e",
                                                    cot_how      = "m-r",
                                                    cot_callsign = str(result.id),
                                                    cot_id       = "object", 
                                                    team_name    = "detector", 
                                                    team_role    = "obj detection",
                                                    cot_lat      = obj_lat,
                                                    cot_lon      = obj_lon,
                                                    img_call     = self.encoded_img,
                                                    img_w        = self.img_w,
                                                    img_h        = self.img_h ))

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
            #
            rospy.loginfo(cot_xml)
            rospy.logdebug("COT XML:\n%s" %(cot_xml))
            if (len(cot_xml)>1):
                self.takmsg_tree = ET.fromstring(cot_xml)
        except:
            rospy.logdebog("Read Cot failed: %s" % (sys.exc_info()[0]))
            
    def parse_takmsg_air(self):
        fiveline = self.takmsg_tree.find("./detail/fiveline")
        #rospy.loginfo("fiveline:%s" %fiveline)
        if not(fiveline in (-1, None)):
            tgt_num = fiveline.attrib['fiveline_target_number']
            # If this is a goto location then publish it as a go to goal.
            # Assumes utm is the global frame.
            rospy.loginfo('--- GoTo Mission Received ---')
            rospy.loginfo(tgt_num)
            
            # Input: 'U99' + 'desired heading angle (3 digits)'                        
            if (tgt_num[0:3] == 'U99'):
                try:
                    this_uid = self.takmsg_tree.get("uid")
                    tgt_lat = self.takmsg_tree.find("./point").attrib['lat']
                    tgt_lon = self.takmsg_tree.find("./point").attrib['lon']
                except Exception as e:
                    rospy.logwarn("----- Recieved ATAK Message and it is not a move to command -----"+ str(e))


                (tgt_zone, tgt_utm_e, tgt_utm_n) = LLtoUTM(23, float(tgt_lat), float(tgt_lon))
                
                #TODO: Remove start_lat/lon. Replace it with frame transformation using utm to odom in launch file.
                (start_zone, start_utm_x, start_utm_y) = LLtoUTM(23, float(self.start_lat), float(self.start_lon))

                #
                tgt_odom_x = tgt_utm_e - start_utm_x
                tgt_odom_y = tgt_utm_n - start_utm_y
                
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped.header.stamp = rospy.Time.now()
                goal_pose_stamped.header.frame_id = self.map_frame
                
                # Target Assignment              
                msg.pose.position.x = tgt_odom_x
                msg.pose.position.y = tgt_odom_y
                #TODO: allow to change alt by using tgt_num
                msg.pose.position.z = self.tgt_alt
                               
                # current utm
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

                #TODO Consider the current heading angle.
                else:
                    #calculate heading angle
                    x_yaw = tgt_utm_e - crnt_utm_x
                    y_yaw = tgt_utm_n - crnt_utm_y
                    yaw = math.atan2(y_yaw,x_yaw)/math.pi*180
#                    yaw = yaw - rot_euler[2]
                    rot = Rotation.from_euler('xyz', [0, 0, yaw], degrees=True)
                    rot_quat = rot.as_quat()
#                    rospy.loginfo('++++++++++ yaw: %f , z: %f, w: %f' %(yaw, rot_quat[2], rot_quat[3]))
                
                    msg.pose.orientation.z = rot_quat[2]
                    msg.pose.orientation.w = rot_quat[3]
               
#                rospy.loginfo(msg)                
                self.goal_pub.publish(msg)
                rospy.loginfo("----- !!(SIMULATION)!! Recieved ATAK Message from UID: %s, saying move to lat/long of %s, %s and map location %s, %s" %(this_uid, tgt_lat,tgt_lon, tgt_odom_x,tgt_odom_y))


    def parse_takmsg_path_air(self):
        try:
            contact = self.takmsg_tree.find('./detail/contact')
            if not(contact in (-1, None)):
                path = contact.attrib['callsign']
                #rospy.loginfo("fiveline:%s" %fiveline)
                if path.find('Route') == 0:
                    rospy.loginfo('--- Route Mission Received ---')

                    robot_path = Path()
                    robot_path.header.stamp = rospy.Time.now()
                    robot_path.header.frame_id = self.map_frame

                    waypoints = self.takmsg_tree.findall('./detail/link')
                    this_uid = self.takmsg_tree.get("uid")

                    for wp in waypoints:
                        tgt_pts = wp.attrib['point'].split(",")
                        tgt_lat = tgt_pts[0]
                        tgt_lon = tgt_pts[1]

                        (tgt_zone, tgt_utm_e, tgt_utm_n) = LLtoUTM(23, float(tgt_lat), float(tgt_lon))

                        #TODO: Remove start_lat/lon. Replace it with frame transformation using utm to odom in launch file.
                        (start_zone, start_utm_x, start_utm_y) = LLtoUTM(23, float(self.start_lat), float(self.start_lon))

                        #
                        tgt_odom_x = tgt_utm_e - start_utm_x
                        tgt_odom_y = tgt_utm_n - start_utm_y
                        
                        wp_pose = PoseStamped()
                        wp_pose.header = robot_path.header
                        
                        wp_pose.pose.position.x = tgt_odom_x
                        wp_pose.pose.position.y = tgt_odom_y
                        #TODO: allow to change alt by using tgt_num
                        wp_pose.pose.position.z = self.tgt_alt
                        
                        # current utm
                        crnt_pose = self.tf1_listener.lookupTransform('utm', self.baselink_frame, rospy.Time(0))
                        crnt_utm_x = crnt_pose[0][0]
                        crnt_utm_y = crnt_pose[0][1]

                        #calculate heading angle
                        x_yaw = tgt_utm_e - crnt_utm_x
                        y_yaw = tgt_utm_n - crnt_utm_y
                        yaw = math.atan2(y_yaw,x_yaw)/math.pi*180
#                        yaw = yaw - rot_euler[2]
                        rot = Rotation.from_euler('xyz', [0, 0, yaw], degrees=True)
                        rot_quat = rot.as_quat()
#                        rospy.loginfo('++++++++++ yaw: %f , z: %f, w: %f' %(yaw, rot_quat[2], rot_quat[3]))
                    
                        wp_pose.pose.orientation.z = rot_quat[2]
                        wp_pose.pose.orientation.w = rot_quat[3]
                        
                        robot_path.poses.append(wp_pose)

                    self.path_pub.publish(robot_path)
                    
                    rospy.loginfo("----- <ROUTE> : Recieved ATAK Message from UID: %s, saying move to lat/long of %s, %s and map location %s, %s" %(this_uid, tgt_lat,tgt_lon, tgt_odom_x,tgt_odom_y))

        except Exception as e:
            rospy.logwarn("----- Recieved ATAK Message and it is not a move to command -----"+ str(e))


    def robot_pose_to_tak(self):
        # Get current position in global frame        
        crnt_pose = self.tf1_listener.lookupTransform('utm', self.baselink_frame, rospy.Time(0))
        crnt_utm_x = crnt_pose[0][0]
        crnt_utm_y = crnt_pose[0][1]
        
        (crnt_lat,crnt_long) = UTMtoLL(23, crnt_utm_y, crnt_utm_x, self.zone) # 23 is WGS-84.
        
        # Send the current position to the TAK Server  
        #rospy.loginfo("latlong: %.7f,%.7f baselinkg is: %s"%(crnt_latitude,crnt_longitude, self.baselink_frame))    
        self.takserver.send(mkcot.mkcot(cot_identity     = "friend", 
                                           cot_stale     = 1,
                                           cot_type      = "a-f-G-M-F-Q",
                                           cot_how       = "m-g",
                                           cot_callsign  = self.my_callsign, 
                                            cot_id       = self.my_uid, 
                                            team_name    = self.my_team_name, 
                                            team_role    = self.my_team_role,
                                            cot_lat      = crnt_lat,
                                            cot_lon      = crnt_long ))
        
    def image_cb(self, data):
        cv_img = self.img_bridge.imgmsg_to_cv2(data, "bgr8")

        self.img_h = cv_img.shape[0]/2
        self.img_w = cv_img.shape[1]/2

        cv_img = cv2.resize(cv_img, (int(self.img_w), int(self.img_h)))
        
        result, final_img = cv2.imencode(".jpeg", cv_img)
        
        # type bool. (Converted to str in mkcot_beta)
        self.encoded_img = base64.b64encode(final_img) 
        #print(self.encoded_img)



if __name__ == '__main__':
    try:
        rospy.init_node("atak_bridge")
        bridge = AtakBridge()
        bridge.takserver_start()
        loop_hz = 10
        rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the timeout.
        while not rospy.is_shutdown():
            bridge.takserver_read()
            bridge.parse_takmsg_air()
            bridge.robot_pose_to_tak()

            bridge.parse_takmsg_path_air()

            rate.sleep()

        bridge.takserver_shutdown()
    except rospy.ROSInterruptException:
        pass





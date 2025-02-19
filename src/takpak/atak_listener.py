#!/usr/bin/env python3

import os
import ssl
import socket
import xml.etree.ElementTree as ET
import csv
import rospy
from takpak.takcot import takcot

class AtakListener:
    def __init__(self, ip_address, port, cert_path, key_path, password):
        self.ip_address = ip_address
        self.port = port
        self.cert_path = os.path.expanduser(cert_path)
        self.key_path = os.path.expanduser(key_path)
        self.password = password
        self.sock = None
        self.takserver = takcot()

    def connect(self):
        try:
            # Check if the certificate and key files exist
            if not os.path.isfile(self.cert_path):
                rospy.logerr(f"Certificate file not found: {self.cert_path}")
                return False
            if not os.path.isfile(self.key_path):
                rospy.logerr(f"Key file not found: {self.key_path}")
                return False

            # Connect to the TAK server
            self.sock = self.takserver.open(self.ip_address, self.port, self.cert_path, self.key_path, self.password)
            if self.sock:
                rospy.loginfo(f"Connected successfully to {self.ip_address}:{self.port}")
                return True
            else:
                rospy.logerr(f"Failed to connect to {self.ip_address}:{self.port}")
                return False
        except Exception as e:
            rospy.logerr(f"Cannot connect to {self.ip_address}:{self.port}. Error: {str(e)}")
            return False

    def listen(self):
        try:
            while not rospy.is_shutdown():
                data = self.sock.recv(2048)
                if data:
                    self.process_message(data)
        except Exception as e:
            rospy.logerr(f"Error while listening: {str(e)}")

    def process_message(self, data):
        try:
            root = ET.fromstring(data)
            # Extract shape information from the message
            # This is an example, adjust according to your message structure
            for shape in root.findall(".//shape"):
                shape_type = shape.get("type")
                coordinates = shape.find("coordinates").text
                self.write_to_csv(shape_type, coordinates)
                rospy.loginfo(f"Received shape: {shape_type} with coordinates: {coordinates}")
        except Exception as e:
            rospy.logerr(f"Failed to process message: {str(e)}")

    def write_to_csv(self, shape_type, coordinates):
        with open('fly_zones.csv', mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([shape_type, coordinates])
        rospy.loginfo(f"Written to CSV: {shape_type}, {coordinates}")

if __name__ == "__main__":
    rospy.init_node('atak_listener')
    ip_address = rospy.get_param('~tak_ip', 'hermes.westpoint.edu')
    port = rospy.get_param('~tak_port', 8089)
    cert_path = rospy.get_param('~cert_path', '~/catkin_ws/src/atak_bridge/src/user2.pem')
    key_path = rospy.get_param('~key_path', '~/catkin_ws/src/atak_bridge/src/user2.key')
    password = rospy.get_param('~password', 'atakatak')

    listener = AtakListener(ip_address, port, cert_path, key_path, password)
    if listener.connect():
        listener.listen()
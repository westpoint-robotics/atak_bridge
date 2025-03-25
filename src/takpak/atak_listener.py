#!/usr/bin/env python3

import os
import ssl
import socket
import xml.etree.ElementTree as ET
import csv
import rospy
import pytak
import select
import threading
import time
import pandas as pd

class AtakListener:
    def __init__(self, ip_address, port, cert_path, key_path, password):
        self.ip_address = ip_address
        self.port = port
        self.cert_path = os.path.expanduser(cert_path)
        self.key_path = os.path.expanduser(key_path)
        self.password = password
        self.sock = None
        self.ssl_context = self.create_ssl_context()

        # Clear the CSV file at the start
        self.clear_csv_file()

    def create_ssl_context(self):
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.load_cert_chain(certfile=self.cert_path, keyfile=self.key_path, password=self.password)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        return context

    def clear_csv_file(self):
        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        csv_file_path = os.path.join(script_dir, 'fly_zones.csv')
        # Open the file in write mode to clear it and write headers
        with open(csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            #writer.writerow(["Type", "Vertices"])  # Write headers
        rospy.loginfo(f"Cleared CSV file at: {csv_file_path}")

    def connect(self):
        rospy.loginfo("Attempting to connect to ATAK server...")
        try:
            # Check if the certificate and key files exist
            if not os.path.isfile(self.cert_path):
                rospy.logerr(f"Certificate file not found: {self.cert_path}")
                return False
            if not os.path.isfile(self.key_path):
                rospy.logerr(f"Key file not found: {self.key_path}")
                return False

            # Creating a standard socket
            self.sock = socket.create_connection((self.ip_address, self.port))
            self.sock = self.ssl_context.wrap_socket(self.sock, server_hostname=self.ip_address)
            self.sock.setblocking(False)  # Set the socket to non-blocking mode
            rospy.loginfo(f"Connected successfully to {self.ip_address}:{self.port}")
            return True
        except ssl.SSLCertVerificationError as e:
            rospy.logerr(f"Cert validation failed: {str(e)}")
            self.sock = None
            return False
        except Exception as e:
            rospy.logerr(f"Cannot connect to {self.ip_address}:{self.port}. Error: {str(e)}")
            self.sock = None
            return False

    def listen(self):
        rospy.loginfo("Starting to listen for incoming messages...")
        try:
            while not rospy.is_shutdown():
                try:
                    # Use select to wait for incoming data
                    ready_to_read, _, _ = select.select([self.sock], [], [], 1.0)
                    if ready_to_read:
                        data = self.sock.recv(2048)
                        if data:
                            rospy.loginfo(f"Data received: {data}")
                            self.process_message(data)
                        else:
                            rospy.logwarn("No data received, continuing to listen...")
                except Exception as e:
                    rospy.logerr(f"Error while listening: {str(e)}")
                    break
        except Exception as e:
            rospy.logerr(f"Error in listen loop: {str(e)}")

    def process_message(self, data):
        try:
            rospy.loginfo(f"Received data: {data}")
            root = ET.fromstring(data)
            # Check if the message is for "Fly" or "NoFly"
            contact = root.find(".//contact")
            if contact is not None:
                rospy.loginfo(f"Contact element found: {ET.tostring(contact)}")
                callsign = contact.get("callsign").lower()
                if callsign in ["fly", "nofly"]:
                    rospy.loginfo(f"Message received for {callsign.capitalize()}")
                    # Extract shape information from the message
                    vertices = []
                    for link in root.findall(".//link"):
                        point = link.get("point")
                        vertices.append(point)
                    self.write_to_csv(callsign.capitalize(), vertices)
                    rospy.loginfo(f"Received shape with vertices: {vertices}")
                elif callsign == "start":
                    rospy.loginfo("Start marker received, setting as start point")
                    self.update_csv_with_marker("start", root)
                elif callsign == "end":
                    rospy.loginfo("End marker received, setting as end point")
                    self.update_csv_with_marker("end", root)
                else:
                    rospy.loginfo(f"Message received but not for Fly, NoFly, Start, or End, callsign: {callsign}")
            else:
                rospy.loginfo("No contact element found in the message")
        except Exception as e:
            rospy.logerr(f"Failed to process message: {str(e)}")

    def update_csv_with_marker(self, marker_type, root):
        point = root.find(".//point")
        if point is not None:
            lat = point.get("lat")
            lon = point.get("lon")
            new_point = f"{lat},{lon}\n"

            # Get the directory of the current script
            script_dir = os.path.dirname(os.path.realpath(__file__))
            csv_file_path = os.path.join(script_dir, 'fly_zones.csv')

            # Read the existing CSV file
            with open(csv_file_path, mode='r') as file:
                existing_data = file.readlines()

            if marker_type == "start":
                # Update the first point
                existing_data[0] = new_point
            elif marker_type == "end":
                # Update the last point
                existing_data[-1] = new_point

            # Ensure there is a separating line after the updated point
            if existing_data[1] != "\n":
                existing_data.insert(1, "\n")
            if existing_data[-2] != "\n":
                existing_data.insert(-1, "\n")

            # Write the updated data back to the CSV file
            with open(csv_file_path, mode='w', newline='') as file:
                file.writelines(existing_data)

            rospy.loginfo(f"Updated CSV with {marker_type} point: {new_point.strip()}")

    def write_to_csv(self, shape_type, vertices):
        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.realpath(__file__))
        csv_file_path = os.path.join(script_dir, 'fly_zones.csv')
        rospy.loginfo(f"Writing to CSV file at: {csv_file_path}")

        # Read the existing CSV file if it exists
        if os.path.exists(csv_file_path):
            with open(csv_file_path, mode='r') as file:
                existing_data = file.readlines()
        else:
            existing_data = []

        # Prepare new data to be written
        new_data = []
        for point in vertices:
            lat, lon = point.split(',')[:2]  # Extract lat and lon from the point
            new_data.append(f"{lat},{lon}\n")
        new_data.append("\n")  # Add an empty row after each shape

        # Write data to CSV file
        with open(csv_file_path, mode='w', newline='') as file:
            if shape_type == "Fly":
                file.writelines(new_data + existing_data)  # Write Fly zones before existing data
            else:
                file.writelines(existing_data + new_data)  # Append NoFly zones after existing data

        rospy.loginfo(f"Written to CSV: {shape_type} with vertices: {vertices}")

    def send_minesweeper_icon(self):
        # Coordinates for West Point
        lat = 41.3911146
        lon = -73.9530166

        # CoT message for MineSweeper icon
        cot_message = f'''
        <event version="2.0" uid="MineSweeper" type="a-f-G-U-C" time="2025-02-04T15:05:59Z" start="2025-02-04T15:05:59Z" stale="2025-02-04T15:06:05Z" how="m-g">
            <point lat="{lat}" lon="{lon}" hae="9999999" ce="9999999.0" le="9999999.0" />
            <detail>
                <contact endpoint="*:-1:stcp" callsign="MineSweeper" />
                <precisionlocation altsrc="GPS" geopointsrc="GPS" />
                <__group role="Team Member" name="Cyan" />
                <takv os="1" platform="atak_listener" version="1.1.0" />
                <color argb="-1" />
            </detail>
        </event>
        '''

        self.send_cot_message(cot_message)

    def send_cot_message(self, cotdata):
        try:
            self.sock.settimeout(0.5)  # 0 is non-blocking
            if isinstance(cotdata, str):
                cotdata = cotdata.encode('utf-8')  # Ensure data is encoded if it's a string
            
            sentdata = self.sock.send(cotdata)
            if sentdata != len(cotdata):
                rospy.logerr("Socket Send mismatch")
                raise Exception("Socket Send mismatch")
            rospy.loginfo("Data sent successfully")
        except socket.timeout as e:
            rospy.logerr(f"Socket Timeout: {str(e)}")
            raise Exception("Socket Timeout")
        except ssl.SSLZeroReturnError as e:
            rospy.logerr(f"SSL connection has been closed (EOF): {str(e)}")
            raise Exception("SSL connection has been closed (EOF)")
        except Exception as e:
            rospy.logerr(f"Send data failed: {str(e)}")
            raise Exception("Send Failed")

    def start_sending_minesweeper_icon(self, interval=0.5):
        def send_periodically():
            while not rospy.is_shutdown():
                self.send_minesweeper_icon()
                time.sleep(interval)

        thread = threading.Thread(target=send_periodically)
        thread.daemon = True
        thread.start()

if __name__ == "__main__":
    rospy.init_node('atak_listener')
    ip_address = rospy.get_param('~tak_ip', 'hermes.westpoint.edu')
    port = rospy.get_param('~tak_port', 8089)
    cert_path = rospy.get_param('~cert_path', '~/catkin_ws/src/atak_bridge/src/user2.pem')
    key_path = rospy.get_param('~key_path', '~/catkin_ws/src/atak_bridge/src/user2.key')
    password = rospy.get_param('~password', 'atakatak')

    listener = AtakListener(ip_address, port, cert_path, key_path, password)
    if listener.connect():
        listener.start_sending_minesweeper_icon()  # Periodically send the MineSweeper icon at West Point
        listener.listen()
    else:
        rospy.logerr("Failed to connect to the ATAK server.")
        rospy.signal_shutdown("Failed to connect to the ATAK server.")
#!/usr/bin/env python3

import os
import csv
import socket
import ssl
import time
import rospy

class PathPlotter:
    def __init__(self, ip_address, port, cert_path, key_path, password):
        self.ip_address = ip_address
        self.port = port
        self.cert_path = cert_path
        self.key_path = key_path
        self.password = password
        self.sock = None
        self.ssl_context = self.create_ssl_context()

    def create_ssl_context(self):
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.load_cert_chain(certfile=self.cert_path, keyfile=self.key_path, password=self.password)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        return context

    def connect(self):
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

    def plot_path(self, csv_file_path):
        with open(csv_file_path, mode='r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row
            for row in reader:
                lat, lon = row
                cot_message = f'''
                <event version="2.0" uid="PathPoint-{lat}-{lon}" type="a-f-G-U-C" time="{time.strftime('%Y-%m-%dT%H:%M:%SZ')}" start="{time.strftime('%Y-%m-%dT%H:%M:%SZ')}" stale="{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(time.time() + 60))}" how="m-g">
                    <point lat="{lat}" lon="{lon}" hae="9999999" ce="9999999.0" le="9999999.0" />
                    <detail>
                        <contact endpoint="*:-1:stcp" callsign="PathPoint" />
                        <precisionlocation altsrc="GPS" geopointsrc="GPS" />
                        <__group role="Team Member" name="Cyan" />
                        <takv os="1" platform="path_plotter" version="1.1.0" />
                        <color argb="-1" />
                        <strokeWeight value="1.0" />
                    </detail>
                </event>
                '''
                self.send_cot_message(cot_message)
                time.sleep(0.01)  # Small delay to simulate movement and avoid overwhelming the server

if __name__ == "__main__":
    rospy.init_node('path_plotter')
    ip_address = rospy.get_param('~tak_ip', 'hermes.westpoint.edu')
    port = rospy.get_param('~tak_port', 8089)
    cert_path = rospy.get_param('~cert_path', '~/catkin_ws/src/atak_bridge/src/user2.pem')
    key_path = rospy.get_param('~key_path', '~/catkin_ws/src/atak_bridge/src/user2.key')
    password = rospy.get_param('~password', 'atakatak')
    csv_file_path = rospy.get_param('~csv_file_path', '~/catkin_ws/src/atak_bridge/src/takpak/Output_Path.csv')

    plotter = PathPlotter(ip_address, port, cert_path, key_path, password)
    if plotter.connect():
        plotter.plot_path(csv_file_path)  # Plot the path from the CSV file
    else:
        rospy.logerr("Failed to connect to the ATAK server.")
        rospy.signal_shutdown("Failed to connect to the ATAK server.")
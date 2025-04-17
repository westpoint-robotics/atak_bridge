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
import tkinter as tk
from tkinter import messagebox

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
        self.clear_csv_files()

        # Counters for Fly, NoFly, Start, and End points
        self.fly_count = 0
        self.nofly_count = 0
        self.start_count = 0
        self.end_count = 0

        # Callback for updating the GUI
        self.update_gui_callback = None

    def set_update_gui_callback(self, callback):
        """Set the callback function to update the GUI."""
        self.update_gui_callback = callback

    def create_ssl_context(self):
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.load_cert_chain(certfile=self.cert_path, keyfile=self.key_path, password=self.password)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        return context

    def clear_csv_files(self):
        """Clear all CSV files at the start."""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        files = ['start_point.csv', 'fly_zones.csv', 'nofly_zones.csv', 'end_point.csv', 'User_input_total.csv']
        for file_name in files:
            file_path = os.path.join(script_dir, file_name)
            with open(file_path, mode='w', newline='') as file:
                pass  # Clear the file
        rospy.loginfo("Cleared all CSV files.")

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
            contact = root.find(".//contact")
            if contact is not None:
                callsign = contact.get("callsign").lower()
                if callsign == "fly":
                    self.fly_count += 1
                    rospy.loginfo(f"Fly zone received. Total Fly zones: {self.fly_count}")
                    vertices = self.extract_vertices(root)
                    self.write_to_csv("Fly", vertices)
                    for vertex in vertices:
                        lat, lon = vertex.split(",")[:2]
                        if self.update_gui_callback:
                            self.update_gui_callback("Fly", lat, lon)
                elif callsign == "nofly":
                    self.nofly_count += 1
                    rospy.loginfo(f"NoFly zone received. Total NoFly zones: {self.nofly_count}")
                    vertices = self.extract_vertices(root)
                    self.write_to_csv("NoFly", vertices)
                    for vertex in vertices:
                        lat, lon = vertex.split(",")[:2]
                        if self.update_gui_callback:
                            self.update_gui_callback("NoFly", lat, lon)
                elif callsign == "start":
                    self.start_count += 1
                    rospy.loginfo(f"Start point received. Total Start points: {self.start_count}")
                    self.update_csv_with_marker("start", root)
                    point = root.find(".//point")
                    if point is not None:
                        lat = point.get("lat")
                        lon = point.get("lon")
                        if self.update_gui_callback:
                            self.update_gui_callback("Start", lat, lon)
                elif callsign == "end":
                    self.end_count += 1
                    rospy.loginfo(f"End point received. Total End points: {self.end_count}")
                    self.update_csv_with_marker("end", root)
                    point = root.find(".//point")
                    if point is not None:
                        lat = point.get("lat")
                        lon = point.get("lon")
                        if self.update_gui_callback:
                            self.update_gui_callback("End", lat, lon)

                # Update the GUI counters
                if self.update_gui_callback:
                    self.update_gui_callback(self.fly_count, self.nofly_count, self.start_count, self.end_count)
        except Exception as e:
            rospy.logerr(f"Failed to process message: {str(e)}")

    def extract_vertices(self, root):
        """Extract vertices from the XML data."""
        vertices = []
        for link in root.findall(".//link"):
            point = link.get("point")
            if point:
                rospy.loginfo(f"Extracted point: {point}")
                vertices.append(point)
        return vertices

    def update_csv_with_marker(self, marker_type, root):
        point = root.find(".//point")
        if point is not None:
            lat = point.get("lat")
            lon = point.get("lon")
            new_point = f"{lat},{lon}\n"  # Use comma-separated values

            # Determine the file to write to
            script_dir = os.path.dirname(os.path.realpath(__file__))
            if marker_type == "start":
                csv_file_path = os.path.join(script_dir, 'start_point.csv')
            elif marker_type == "end":
                csv_file_path = os.path.join(script_dir, 'end_point.csv')

            # Write the point to the respective file
            with open(csv_file_path, mode='w', newline='') as file:
                file.write(new_point)

            rospy.loginfo(f"Updated {marker_type} point in {csv_file_path}")


    def write_to_csv(self, shape_type, vertices):
        # Determine the file to write to
        script_dir = os.path.dirname(os.path.realpath(__file__))
        if shape_type == "Fly":
            csv_file_path = os.path.join(script_dir, 'fly_zones.csv')
        elif shape_type == "NoFly":
            csv_file_path = os.path.join(script_dir, 'nofly_zones.csv')

        # Write the vertices to the respective file
        with open(csv_file_path, mode='a', newline='') as file:  # Append mode
            for point in vertices:
                lat, lon = point.split(',')[:2]
                file.write(f"{lat},{lon}\n")  # Write the point
            if shape_type == "NoFly":
                file.write("\n")  # Add a blank line for separation

        rospy.loginfo(f"Written {shape_type} zones to {csv_file_path}")


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

    def combine_csv_files(self):
        """Combine the separate CSV files into a single file with separation lines."""
        script_dir = os.path.dirname(os.path.realpath(__file__))
        combined_csv_path = os.path.join(script_dir, 'User_input_total.csv')

        # Paths to the individual CSV files
        start_point_path = os.path.join(script_dir, 'start_point.csv')
        fly_zones_path = os.path.join(script_dir, 'fly_zones.csv')
        nofly_zones_path = os.path.join(script_dir, 'nofly_zones.csv')
        end_point_path = os.path.join(script_dir, 'end_point.csv')

        # Open the combined file for writing
        with open(combined_csv_path, mode='w', newline='') as combined_file:
            # Write Start point
            if os.path.exists(start_point_path):
                with open(start_point_path, mode='r') as file:
                    combined_file.writelines(file.readlines())
                combined_file.write("\n")  # Add a blank line for separation

            # Write Fly zones
            if os.path.exists(fly_zones_path):
                with open(fly_zones_path, mode='r') as file:
                    combined_file.writelines(file.readlines())
                combined_file.write("\n")  # Add a blank line for separation

            # Write NoFly zones
            if os.path.exists(nofly_zones_path):
                with open(nofly_zones_path, mode='r') as file:
                    combined_file.writelines(file.readlines())
                combined_file.write("\n")  # Add a blank line for separation

            # Write End point
            if os.path.exists(end_point_path):
                with open(end_point_path, mode='r') as file:
                    combined_file.writelines(file.readlines())

        rospy.loginfo(f"Combined CSV file created at: {combined_csv_path}")


class AtakGUI:
    def __init__(self, root, listener):
        self.root = root
        self.listener = listener
        self.listener.set_update_gui_callback(self.update_chart)

        # Set the window title and size
        self.root.title("ATAK Listener GUI")
        self.root.geometry("1200x900")  # Increased window size
        self.root.configure(bg="#4b5320")  # Army green background color

        # Define the font style
        self.font_style = ("Stencil", 16, "bold")  # Smaller font size
        self.font_color = "#FFFF00"  # Yellow font color

        # Main frame for counters and points
        self.main_frame = tk.Frame(root, bg="#4b5320")
        self.main_frame.pack(pady=20)

        # Fly counter and points
        self.fly_frame = tk.Frame(self.main_frame, bg="#4b5320")
        self.fly_frame.grid(row=0, column=0, padx=20)
        self.fly_label = tk.Label(
            self.fly_frame, text="Fly: 0", font=self.font_style, bg="#4b5320", fg=self.font_color
        )
        self.fly_label.pack()
        self.fly_points_frame = tk.Frame(self.fly_frame, bg="#4b5320")
        self.fly_points_frame.pack()

        # NoFly counter and points
        self.nofly_frame = tk.Frame(self.main_frame, bg="#4b5320")
        self.nofly_frame.grid(row=0, column=1, padx=20)
        self.nofly_label = tk.Label(
            self.nofly_frame, text="NoFly: 0", font=self.font_style, bg="#4b5320", fg=self.font_color
        )
        self.nofly_label.pack()
        self.nofly_points_frame = tk.Frame(self.nofly_frame, bg="#4b5320")
        self.nofly_points_frame.pack()

        # Start counter and points
        self.start_frame = tk.Frame(self.main_frame, bg="#4b5320")
        self.start_frame.grid(row=0, column=2, padx=20)
        self.start_label = tk.Label(
            self.start_frame, text="Start: 0", font=self.font_style, bg="#4b5320", fg=self.font_color
        )
        self.start_label.pack()
        self.start_points_frame = tk.Frame(self.start_frame, bg="#4b5320")
        self.start_points_frame.pack()

        # End counter and points
        self.end_frame = tk.Frame(self.main_frame, bg="#4b5320")
        self.end_frame.grid(row=0, column=3, padx=20)
        self.end_label = tk.Label(
            self.end_frame, text="End: 0", font=self.font_style, bg="#4b5320", fg=self.font_color
        )
        self.end_label.pack()
        self.end_points_frame = tk.Frame(self.end_frame, bg="#4b5320")
        self.end_points_frame.pack()

        # Bottom frame for buttons and status
        self.bottom_frame = tk.Frame(root, bg="#4b5320")
        self.bottom_frame.pack(side=tk.BOTTOM, pady=20)

        # Start button
        self.start_button = tk.Button(
            self.bottom_frame,
            text="Start",
            command=self.start_listener,
            font=self.font_style,
            bg="#4b5320",  # Army green background
            fg=self.font_color,  # Yellow text
            width=20,
            height=2,
        )
        self.start_button.pack(side=tk.LEFT, padx=10)

        # End button (hidden initially)
        self.end_button = tk.Button(
            self.bottom_frame,
            text="End",
            command=self.end_program,
            font=self.font_style,
            bg="#4b5320",
            fg=self.font_color,
            width=20,
            height=2,
        )
        self.end_button.pack(side=tk.LEFT, padx=10)
        self.end_button.pack_forget()  # Hide the End button initially

        # Status label
        self.status_label = tk.Label(
            self.bottom_frame,
            text="",
            font=self.font_style,
            bg="#4b5320",
            fg=self.font_color,
        )
        self.status_label.pack(side=tk.LEFT, padx=10)

    def start_listener(self):
        self.animate_button(self.start_button)  # Animate the Start button
        self.start_button.config(state=tk.DISABLED)
        self.status_label.config(text="Status: Listening for messages...")
        threading.Thread(target=self.run_listener, daemon=True).start()

    def run_listener(self):
        if self.listener.connect():
            threading.Thread(target=self.listener.start_sending_minesweeper_icon, daemon=True).start()
            self.listener.listen()
        else:
            messagebox.showerror("Error", "Failed to connect to ATAK server.")
            self.root.quit()

    def update_chart(self, point_type_or_counts, lat=None, lon=None):
        """Update the chart with the latest counts and display all points."""
        if isinstance(point_type_or_counts, str):  # If it's a point type, display the point
            point_label = tk.Label(
                self.root,
                text=f"{lat}, {lon}",
                font=("Arial", 10),
                bg="#4b5320",
                fg="#FFFF00",
            )
            if point_type_or_counts == "Fly":
                point_label.pack(in_=self.fly_points_frame, anchor="w")
                self.fly_label.config(text=f"Fly: {self.listener.fly_count}")
                self.flash_background("#FFD700")  # Flash for Fly
            elif point_type_or_counts == "NoFly":
                point_label.pack(in_=self.nofly_points_frame, anchor="w")
                self.nofly_label.config(text=f"NoFly: {self.listener.nofly_count}")
                self.flash_background("#FF4500")  # Flash for NoFly
            elif point_type_or_counts == "Start":
                point_label.pack(in_=self.start_points_frame, anchor="w")
                self.start_label.config(text=f"Start: {self.listener.start_count}")
                self.flash_background("#32CD32")  # Flash for Start
            elif point_type_or_counts == "End":
                point_label.pack(in_=self.end_points_frame, anchor="w")
                self.end_label.config(text=f"End: {self.listener.end_count}")
                self.flash_background("#1E90FF")  # Flash for End
                self.end_button.pack()  # Show the End button when "End" point is received
        else:  # Otherwise, update the counters
            fly_count, nofly_count, start_count, end_count = point_type_or_counts
            self.fly_label.config(text=f"Fly: {fly_count}")
            self.nofly_label.config(text=f"NoFly: {nofly_count}")
            self.start_label.config(text=f"Start: {start_count}")
            self.end_label.config(text=f"End: {end_count}")

    def flash_background(self, color):
        """Flash the background color to indicate a message was received."""
        original_color = self.root.cget("bg")

        def reset_background():
            if self.root.cget("bg") == color:  # Ensure it resets only if the color hasn't changed
                self.root.configure(bg=original_color)

        self.root.configure(bg=color)  # Flash to the specified color
        self.root.after(500, reset_background)  # Revert after 500ms

    def end_program(self):
        """End the program when the End button is clicked."""
        self.animate_button(self.end_button)  # Animate the End button
        self.listener.combine_csv_files()  # Combine the CSV files
        messagebox.showinfo("Info", "Data collection complete. Combined CSV file created.")
        self.root.quit()

    def animate_button(self, button):
        """Animate a button by changing its background color."""
        original_color = button.cget("bg")
        button.configure(bg="#FFD700")  # Change to gold
        self.root.after(200, lambda: button.configure(bg=original_color))  # Revert after 200ms

if __name__ == "__main__":
    rospy.init_node('atak_listener')
    ip_address = rospy.get_param('~tak_ip', 'hermes.westpoint.edu')
    port = rospy.get_param('~tak_port', 8089)
    cert_path = rospy.get_param('~cert_path', '~/catkin_ws/src/atak_bridge/src/user2.pem')
    key_path = rospy.get_param('~key_path', '~/catkin_ws/src/atak_bridge/src/user2.key')
    password = rospy.get_param('~password', 'atakatak')

    listener = AtakListener(ip_address, port, cert_path, key_path, password)
    root = tk.Tk()
    gui = AtakGUI(root, listener)
    root.mainloop()
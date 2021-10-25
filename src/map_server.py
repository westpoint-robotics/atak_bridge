#!/usr/bin/env python2

import SimpleHTTPServer 
import SocketServer 
import os
import rospy

# TODO handle multiple requests in threads
# TODO shutdown and start on command from the robot

robot_name = rospy.get_param('~name', "warty")
port = rospy.get_param('~fileserver_port', 8000)
directory = rospy.get_param('~fileserver_directory', '/tmp/atak_fileserver')

print(os.path.exists(directory))
if (not os.path.exists(directory)):
    os.makedirs(directory)
    print("Creating file server directory at:",directory)

os.chdir(directory)
Handler = SimpleHTTPServer.SimpleHTTPRequestHandler 

httpd = SocketServer.TCPServer(("", port), Handler) 

print "serving at port", port 
httpd.serve_forever()


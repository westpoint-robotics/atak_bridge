#!/usr/bin/env python2

import SimpleHTTPServer 
import SocketServer 
import os
import shutil
import rospy
import yaml
import cv2
import simplekml
from zipfile import ZipFile


input_file_dir='/home/user1/Dropbox/ATAK/MapConvert/jackalmap/'
out_directory = '/tmp/atak_fileserver/ros_map/'

rosmap_yaml=''
# Read map YAML file into a dictionary
with open(input_file_dir+"jackalmap.yaml", 'r') as stream:
    rosmap_yaml = yaml.safe_load(stream)


print(rosmap_yaml)

# Read in map picture file to access its dimensions and convert it into png format.
im = cv2.imread(input_file_dir+"jackalmap.pgm")
cv2.imwrite(input_file_dir+"jackalmap.png", im)

# Determine dimenstion of map in meters
x_meters = im.shape[0] * rosmap_yaml['resolution']
y_meters = im.shape[1] * rosmap_yaml['resolution']
print(x_meters,y_meters)

# Assumption, map orientation is 0, if not so then exit.
# TODO account for the case where the orientation is not zero. This may occur if mapping starts after the robot moves.
if not rosmap_yaml['origin'][2] == 0.0:
    print("Exiting atak map conversion: This only works if the map orientation as shown in the ros yaml file is 0.0")
    exit(-1)

# Establish the four corners of the map in the robot map frame.
# TODO Convert these to lat long in global frame.
ll_ccord = (rosmap_yaml['origin'][0], rosmap_yaml['origin'][1])
ul_ccord = (rosmap_yaml['origin'][0], rosmap_yaml['origin'][1]+y_meters)
ur_ccord = (rosmap_yaml['origin'][0]+x_meters, rosmap_yaml['origin'][1]+y_meters)
lr_ccord = (rosmap_yaml['origin'][0]+x_meters, rosmap_yaml['origin'][1])
print(ll_ccord,ul_ccord,ur_ccord,lr_ccord)

# === Build KMZ file in output directory =================

# Check if output directory exists, if not then create it, then switch to that directory
if (os.path.exists(out_directory)):
    os.remove(out_directory)

os.makedirs(out_directory)
print("Creating file server directory at:",out_directory)
    
os.chdir(out_directory)
os.makedirs('files')
shutil.copy2(input_file_dir+"jackalmap.png","files/map.png")


kml = simplekml.Kml()

ground = kml.newgroundoverlay(name='Robot Map')
ground.icon.href = "files/map.png"
# The coordinates must be specified in counter-clockwise order with the first coordinate corresponding to the lower-left corner
ground.gxlatlonquad.coords=[ll_ccord,lr_ccord,ur_ccord,ul_ccord,]
ground.altitudemode="ClampToGround"
ground.altitude=0

if os.path.exists("map.kml"):
    os.remove("map.kml")

kml.save("map.kml")
with ZipFile('map.kmz', 'w') as zipObj:
    zipObj.write("map.kml")
    zipObj.write("files/map.png")

os.remove("files/map.png")
os.remove("map.kml")
os.rename('map.kmz','files/map.kmz')
os.makedirs('MANIFEST')

output = '<MissionPackageManifest version="2">\n\t<Configuration>\n'
output += '\t\t<Parameter name="uid" value="Robot UID HERE"/>\n'
output += '\t\t<Parameter name="name" value="ROBOT NAME_MAP"/>\n'
output += '\t</Configuration>\n\t<Contents>\n'
output += '\t\t<Content ignore="false" zipEntry="files/map.kmz">\n'
output += '\t\t\t<Parameter name="contentType" value="External GRG Data"/>\n'
output += '\t\t</Content>\n\t</Contents>\n</MissionPackageManifest>'

with open('MANIFEST/manifest.xml', 'w') as outfile:
    outfile.write(output)

os.chdir('..')

shutil.make_archive('ros_map', 'zip', 'ros_map')
shutil.rmtree("ros_map")





'''
image: testmap.png
resolution: 0.1
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0

image : Path to the image file containing the occupancy data; can be absolute, or relative to the location of the YAML file
resolution : Resolution of the map, meters / pixel
origin : The 2-D pose of the lower-left pixel in the map, as (x, y, yaw), with yaw as counterclockwise rotation (yaw=0 means no rotation). Many parts of the system currently ignore yaw.
occupied_thresh : Pixels with occupancy probability greater than this threshold are considered completely occupied.
free_thresh : Pixels with occupancy probability less than this threshold are considered completely free.
negate : Whether the white/black free/occupied semantics should be reversed (interpretation of thresholds is unaffected)



'''

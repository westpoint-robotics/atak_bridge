#!/usr/bin/env python
from __future__ import (absolute_import, division,
                        print_function, unicode_literals)

import rospy
from visualization_msgs.msg import MarkerArray, Marker
import tf

# Setup ROS node
rospy.init_node('obj_2_map', anonymous=True)
rospy.Subscriber("/uav1/detection_localization/out/markers", MarkerArray, object_location_cb)
def object_location_cb(data):
    for item in data: 
        item_marker = Marker()
        if item.id == 1:
            item_marker = item
            
            
        


loop_hz = 20
rate = rospy.Rate(loop_hz) # The rate of the loop is no faster than then the readtimeout.
while not rospy.is_shutdown():
 
    rate.sleep()



    
'''
rostopic info /uav1/detection_localization/out/markers
Type: visualization_msgs/MarkerArray

---
markers: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: ''
    ns: ''
    id: 0
    type: 0
    action: 3
    pose: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0
    scale: 
      x: 0.0
      y: 0.0
      z: 0.0
    color: 
      r: 0.0
      g: 0.0
      b: 0.0
      a: 0.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 7323
      stamp: 
        secs: 1020
        nsecs: 100000000
      frame_id: "uav1/RGBCamera_Mid"
    ns: ''
    id: 0
    type: 5
    action: 0
    pose: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    scale: 
      x: 0.05
      y: 0.0
      z: 0.0
    color: 
      r: 1.0
      g: 0.0
      b: 0.0
      a: 0.699999988079
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: 
      - 
        x: 0.0
        y: 0.0
        z: 0.0
      - 
        x: -56.4332959763
        y: -32.162670691
        z: 76.0318730485
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 7323
      stamp: 
        secs: 1020
        nsecs: 100000000
      frame_id: "uav1/RGBCamera_Mid"
    ns: ''
    id: 1
    type: 2
    action: 0
    pose: 
      position: 
        x: -1.86433341648
        y: -1.06252772756
        z: 2.51179306807
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    scale: 
      x: 0.5
      y: 0.5
      z: 0.5
    color: 
      r: 1.0
      g: 0.0
      b: 0.0
      a: 0.699999988079
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 7323
      stamp: 
        secs: 1020
        nsecs: 100000000
      frame_id: "uav1/RGBCamera_Mid"
    ns: ''
    id: 2
    type: 2
    action: 0
    pose: 
      position: 
        x: -0.282457716763
        y: 0.084744468797
        z: 1.10893815709
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    scale: 
      x: 0.5
      y: 0.5
      z: 0.5
    color: 
      r: 0.0
      g: 0.0
      b: 1.0
      a: 0.699999988079
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
---

=====================================================================================
rostopic info /uav1/detection_localization/out/markers
Type: visualization_msgs/MarkerArray


---
markers: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: ''
    ns: ''
    id: 0
    type: 0
    action: 3
    pose: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0
    scale: 
      x: 0.0
      y: 0.0
      z: 0.0
    color: 
      r: 0.0
      g: 0.0
      b: 0.0
      a: 0.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 2396
      stamp: 
        secs: 333
        nsecs: 616000000
      frame_id: "uav1/RGBCamera_Mid"
    ns: ''
    id: 0
    type: 5
    action: 0
    pose: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    scale: 
      x: 0.05
      y: 0.0
      z: 0.0
    color: 
      r: 1.0
      g: 0.0
      b: 0.0
      a: 0.699999988079
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: 
      - 
        x: 0.0
        y: 0.0
        z: 0.0
      - 
        x: -56.4332959763
        y: -32.162670691
        z: 76.0318730485
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 2396
      stamp: 
        secs: 333
        nsecs: 616000000
      frame_id: "uav1/RGBCamera_Mid"
    ns: ''
    id: 1
    type: 2
    action: 0
    pose: 
      position: 
        x: -1.8707070152
        y: -1.06616019228
        z: 2.52038013783
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    scale: 
      x: 0.5
      y: 0.5
      z: 0.5
    color: 
      r: 1.0
      g: 0.0
      b: 0.0
      a: 0.699999988079
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 2396
      stamp: 
        secs: 333
        nsecs: 616000000
      frame_id: "uav1/RGBCamera_Mid"
    ns: ''
    id: 2
    type: 2
    action: 0
    pose: 
      position: 
        x: -0.282457716763
        y: 0.084744468797
        z: 1.10893815709
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    scale: 
      x: 0.5
      y: 0.5
      z: 0.5
    color: 
      r: 0.0
      g: 0.0
      b: 1.0
      a: 0.699999988079
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
---




'''


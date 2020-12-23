#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped


#587512.4546064622 4582644.3381104795 Window location
#41.39058,-73.95323 robot
#41.39057,-73.95325 window

'''
rostopic echo /husky/worldmodel_rviz/object_markers

rostopic info /husky/worldmodel_rviz/object_markers
Type: visualization_msgs/MarkerArray
'''
rospy.init_node('obj_2_atak', anonymous=True)

tf1_listener = tf.TransformListener()
tf1_listener.waitForTransform("husky/map", "utm", rospy.Time(0), rospy.Duration(3.0));


def oblect_cb(data):
    for item in data.markers:
        if item.ns == "window":
            print(str(item.header.stamp.secs)+ " " + item.ns + " " + str(item.pose.position.x) + " " + str(item.pose.position.y) + " " + str(item.pose.position.z))
            
            obj_pose_stamped = PoseStamped()
            obj_pose_stamped.header = item.header  
            obj_pose_stamped.header.stamp = rospy.Time.now() 
                      
            obj_pose_stamped.pose = item.pose
            obj_pose = tf1_listener.transformPose("utm", obj_pose_stamped)
            print(str(item.header.stamp.secs)+ " " + item.ns + " " + str(obj_pose.pose.position.x) + " " + str(obj_pose.pose.position.y) + " " + str(obj_pose.pose.position.z))
    print("NEXT MSG")
    
    
    

def main():

    rospy.Subscriber("/husky/worldmodel_rviz/object_markers", MarkerArray, oblect_cb)
    rate = rospy.Rate(10) 
    
    
    
    while not rospy.is_shutdown():
        pass
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
        
        
'''

markers: 
  - 
    header: 
      seq: 1663
      stamp: 
        secs: 1608591553
        nsecs: 812165436
      frame_id: "husky/map"
    ns: "window"
    id: 1
    type: 1
    action: 0
    pose: 
      position: 
        x: 1.38879642925
        y: 0.883144925841
        z: -0.370173223144
      orientation: 
        x: -0.519537972719
        y: 0.47932150727
        z: -0.480969623584
        w: 0.51865152922
    scale: 
      x: 1.0
      y: 1.0
      z: 1.0
    color: 
      r: 0.783878028393
      g: 0.990000009537
      b: 0.495000004768
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: True
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 1663
      stamp: 
        secs: 1608591553
        nsecs: 812165436
      frame_id: "husky/map"
    ns: "window"
    id: 16777217
    type: 9
    action: 0
    pose: 
      position: 
        x: 1.38879642925
        y: 0.883144925841
        z: -0.370173223144
      orientation: 
        x: -0.519537972719
        y: 0.47932150727
        z: -0.480969623584
        w: 0.51865152922
    scale: 
      x: 1.0
      y: 1.0
      z: 1.0
    color: 
      r: 0.783878028393
      g: 0.990000009537
      b: 0.495000004768
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: True
    points: []
    colors: []
    text: "window-1"
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 1928
      stamp: 
        secs: 1608591556
        nsecs: 513953385
      frame_id: "husky/map"
    ns: "door"
    id: 3
    type: 1
    action: 0
    pose: 
      position: 
        x: 1.8097261823
        y: -0.904801119098
        z: 0.256656252046
      orientation: 
        x: 0.432963654553
        y: -0.559282120071
        z: 0.561433118674
        w: -0.429579838051
    scale: 
      x: 1.0
      y: 1.0
      z: 1.0
    color: 
      r: 0.990000009537
      g: 0.495000004768
      b: 0.928317129612
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: True
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 1928
      stamp: 
        secs: 1608591556
        nsecs: 513953385
      frame_id: "husky/map"
    ns: "door"
    id: 16777219
    type: 9
    action: 0
    pose: 
      position: 
        x: 1.8097261823
        y: -0.904801119098
        z: 0.256656252046
      orientation: 
        x: 0.432963654553
        y: -0.559282120071
        z: 0.561433118674
        w: -0.429579838051
    scale: 
      x: 1.0
      y: 1.0
      z: 1.0
    color: 
      r: 0.990000009537
      g: 0.495000004768
      b: 0.928317129612
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: True
    points: []
    colors: []
    text: "door-3"
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 1930
      stamp: 
        secs: 1608591556
        nsecs: 680903870
      frame_id: "husky/map"
    ns: "chair"
    id: 4
    type: 1
    action: 0
    pose: 
      position: 
        x: 5.2161424758
        y: 0.475957240424
        z: 0.319180379426
      orientation: 
        x: 0.426384041496
        y: -0.564055660177
        z: 0.56721755077
        w: -0.422258346843
    scale: 
      x: 1.0
      y: 1.0
      z: 1.0
    color: 
      r: 0.495000004768
      g: 0.990000009537
      b: 0.907243967056
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: True
    points: []
    colors: []
    text: ''
    mesh_resource: ''
    mesh_use_embedded_materials: False
  - 
    header: 
      seq: 1930
      stamp: 
        secs: 1608591556
        nsecs: 680903870
      frame_id: "husky/map"
    ns: "chair"
    id: 16777220
    type: 9
    action: 0
    pose: 
      position: 
        x: 5.2161424758
        y: 0.475957240424
        z: 0.319180379426
      orientation: 
        x: 0.426384041496
        y: -0.564055660177
        z: 0.56721755077
        w: -0.422258346843
    scale: 
      x: 1.0
      y: 1.0
      z: 1.0
    color: 
      r: 0.495000004768
      g: 0.990000009537
      b: 0.907243967056
      a: 1.0
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: True
    points: []
    colors: []
    text: "chair-4"
    mesh_resource: ''
    mesh_use_embedded_materials: False
---

'''

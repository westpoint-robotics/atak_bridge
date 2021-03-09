# ROS to ATAK bridge
## Notes
- Based on work found at: https://github.com/pinztrek/takpak.git.
    - The python code found in the takpak directory is modified versions of this code.
    - Some of the python code found outside the takpak directory is based on example code provided by takpak.  

## Requirement
- These nodes require a transform from a global frame such as 'utm' to the robot's base_link frame.   

## WARNING:  
The nodes that convert between UTM and LL are not tested for navigation across UTM zones. This code is intended for development and testing of robots in a single UTM zone. It is not tested in situations where the robot crosses UTM zones.   

#### Example usage
There are launch files compatible with a UGS using ARL Phoenix stack and UAS Mav_Platform stack. If you are using this with other software stacks you may need to modify the messages pulbished. Each version uses a node with similiar code. They are not yet consolidated because the they each use custom messages that are propritery to their software stack.
1. To run this with the Phoenix stack use:  
- `roslaunch atak_bridge grnd_tak_bridge.launch`  
2. To run this with the Mav_Platform stack use:  
- `roslaunch atak_bridge uas_tak_bridge.launch`  
- This node relies on the object class name being published in the detected object topic. This maybe enabled with this parameter:  
    `<param name="/uav1/tflite_ros/use_label_as_id" value="True"/>`  

#### Parameters
- `name` (string, default: 'husky')  
    Used to namespace topics  
- `team_name` (string, default: 'Cyan')  
    This system's team name. This should align with ATAK teams, usually colors.  
- `team_role` (string, default: 'Team Member')  
    This system's role in the team. This should align with team roles.  
- `tak_ip` (string, default: '127.0.0.1')  
    The IP address of the server  
- `tak_port` (string, default: '8088')  
    The port for an unsecure connection to the server.
- `baselink_frame` (string, default: 'base_link')  
    The name of the frame that as at the base of the robot.
- `global_frame` (string, default: '8088')  
    The global frame that is converted to Lat/Long before being sent/recevied from the ATAK server. Usually 'utm'
- `callsign` (string, default: 'default_callsign')  
    The call sign used by this system to identify itself on TAK  
- `uid` (string, default: 'fqdn + uuid')  
    A unique identifier used by TAK to identify this system. This will be automatically set by the system. If manually set, the manual setting will override the automatically generated 'fqdn + uuid'.  

#### Published Topics
- UAS (oop_dcist_air.py): `~/goto_region/goal` (geometry_msgs/PoseStamped): Current location in latitude, longitude, and altitude that a ATAK system is requesting the robot to move to.   
- UGS (oop_dcist_grnd.py): `~/goto_region/goal` (arl_nav_msgs/GotoRegionActionGoal): Current location in latitude, longitude, and altitude that a ATAK system is requesting the robot to move to.   

#### Subscribed Topics
- UAS (oop_dcist_air.py): `~/detection_localization/detections/out/local` (vision_msgs/Detection2DArray): Current location of objects of interest. This is used to update the TAK Server on the position of objects of interest.
- UGS (oop_dcist_grnd.py): `~/worldmodel_rviz/object_markers` (visualization_msgs/MarkerArray): Current location of objects of interest. This is used to update the TAK Server on the position of objects of interest.


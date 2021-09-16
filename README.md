# ROS to ATAK bridge
## Notes
- The ATAK Client code is based on work found at: https://github.com/pinztrek/takpak.git.
    - The python code found in the takpak directory is modified versions of this code.
    - Some of the python code found outside the takpak directory is based on example code provided by takpak.  
- ATAK Tutorials at: https://www.youtube.com/playlist?list=PLD4gdaBHX0b7GpPkuy0mbPaCw9kG3YgfB
- This code expects the ATAK User to have the RRC AtAK Plugin installed on the ATAK device. This is required for the go to goal behavior to work. Th

## Requirement
- ROS TF has a transform from  base_link – odom – UTM. This is used to find the robot position in the global frame.
- The objects found provide their location relative to the odom frame.
- Each object found has a unique name, usually the category name and a number
    - Examples: people01, people02, car01, car02, car03.
    - atak_bridge creates a uid by concatenating the unique name and the robot name. Example: husky1_people02.
    - If the name is not unique, ATAK treats this message as an update to location.

## WARNING:  
The nodes that convert between UTM and LL are not tested for navigation across UTM zones. This code is intended for development and testing of robots in a single UTM zone. It is not tested in situations where the robot crosses UTM zones.   

#### Example usage
There are launch files compatible with a UGS using ARL Phoenix stack and UAS Mav_Platform stack. If you are using this with other software stacks you may need to modify the messages pulbished. Each version uses a node with similiar code. They are not yet consolidated because the they each use custom messages that are propritery to their software stack.
1. To run test code with a simulated robot:  
    - `roslaunch atak_bridge test_tak_bridge.launch`  
2. To run this with another robot:  
    - `roslaunch atak_bridge plugin_tak_bridge.launch`  
    - NOTE: atak_bridge requires a transform from utm to odom frame. To run this with your robot make sure this requirement is met. There is an example in the launch file on how to do this with a static transform publisher.  

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
    The call sign used by this system to identify itself on TAK.   

#### Published Topics
- `~/goal_location` (atak_msgs/PoseDescriptionStamped): The location that the ATAK system is requesting the robot to moves to. This can include orientation and altitude if desired. Current location in latitude, longitude, and altitude that a ATAK system is requesting the robot to move to. The pose is published with a frame_id of UTM. The Description is used to identify if the message is intended for this robot.  


#### Subscribed Topics
- `~/object_locations` (atak_msgs/PoseDescriptionArray): This is a list of object locations and descriptions that should be displayed to ATAK users. This message uses a single header timestamp and frame_id for all objects. 

#### Custom Messages
The atak_bridge uses custom message types to send a position and description of that position in a single message. Below are the message definitions and block diagram can be found at: [Communication Block Diagram](https://github.com/westpoint-robotics/atak_bridge/blob/master/docs/ATAK_Plugin.pdf)
- `atak_bridge/PoseDescription` 
    - `geometry_msgs/Pose` pose
    - `std_msgs/String` description
- `atak_bridge/PoseDescriptionStamped` 
    - `std_msgs/Header` header
    - `atak_bridge/PoseDescription` pose
        - `geometry_msgs/Pose` pose
        - `std_msgs/String` description
- `atak_bridge/PoseDescriptionArray`
    - `std_msgs/Header` header
    - `atak_bridge/PoseDescription[]` pose_list
        - `geometry_msgs/Pose` pose
        - `std_msgs/String` description
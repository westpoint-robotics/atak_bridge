# ROS to ATAK bridge
## Notes
- The ATAK Client code is based on work found at: https://github.com/pinztrek/takpak.git.
    - The python code found in the takpak directory is modified versions of this code.
    - Some of the python code found outside the takpak directory is based on example code provided by takpak.  
- This code expects the ATAK User to have the RRC ATAK Plugin installed on the ATAK device. This is required for the "go to goal" behavior to work.
- Tutorials on generic ATAK usage can be found at: https://www.youtube.com/playlist?list=PLD4gdaBHX0b7GpPkuy0mbPaCw9kG3YgfB

## Requirement
- ROS TF has a transform from  base_link – UTM. This is used to find the robot position in the global frame and the location of detected objects in the global frame.
- The objects found provide their location relative to the frame specified in map_frame parameter.
- Each object found has a unique name, usually the category name and a number
    - Examples: people01, people02, car01, car02, car03.
    - atak_bridge creates a object message uid by concatenating the robot name and the object's unique name. Example: husky1_people02.
    - If the object message uid is not unique, ATAK treats this message as an update to the existing object's location.

## WARNING:  
The nodes that convert between UTM and LL are not tested for navigation across UTM zones. This code is intended for development and testing of robots in a single UTM zone. It is not tested in situations where the robot crosses UTM zones.   

#### Example usage
To use atak-bridge it is typical to write one or two new nodes that are robot specific. These would serve two purposes:
1. From the robot acquire the location and class name for each object of interest and publish it to the atak_bridge in as a list of objects in a atak_bridge/PoseDescriptionArray message type.
2. From the atak_bridge acquires a go to goal location with optional heading and optional altitude. This information is sent to the robot using a robot specific methods. The atak_bridge publishes the go to goal as a message of type atak_bridge/PoseDescriptionStamped. 
3. To run test code with a simulated robot:  
    - `roslaunch atak_bridge test_tak_bridge.launch`  
4. To run this with another robot:  
    - `roslaunch atak_bridge plugin_tak_bridge.launch`
    - run the nodes that communicate with the atak_bridge.  
    - NOTE: atak_bridge requires a transform from utm to base_link frame. To run this with your robot make sure this requirement is met. There is an example in the launch file on how to do this with a static transform publisher.  

#### Parameters
- `name` (string, default: 'warty')  
    Used to namespace topics  
- `callsign` (string, default: 'default_callsign')  
    The call sign used by this system to identify itself on TAK.   
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
- `map_frame` (string, default: 'map')  
    The global frame that is converted to Lat/Long before being sent/recevied from the ATAK server. Usually 'warty/map'
- `robot_msg_uid` (string, default: 'warty1_goto')  
    Used to identify ATAK Messages that are meant for this robot. The robot ignores all incoming atak messages that do not match this string.

#### Published Topics
- `~/goal_location` (atak_msgs/PoseDescriptionStamped): The location that the ATAK system is requesting the robot to moves to. This can include orientation and altitude if desired. Current location in latitude, longitude, and altitude that a ATAK system is requesting the robot to move to. The pose is published with a frame_id of UTM. The Description is used to identify if the message is intended for this robot.

#### Subscribed Topics
- `~/object_locations` (atak_msgs/PoseDescriptionArray): This is a list of object locations and descriptions that should be displayed to ATAK users. This message uses a single header timestamp and frame_id for all objects. It assumed this a list that only contains the objects to be displayed in ATAK, this is typically not all the detected objects. The robot node that publishes this information to the atak_bridge should filter out the unwanted objects. The atak_bridge test_robot.py file has an example of doing this with a python list of desired objects.

#### Custom Messages
The atak_bridge uses custom message types to send a position and description of that position in a single message. Below are the message definitions and block diagram can be found at: [Communication Block Diagram](https://github.com/westpoint-robotics/atak_bridge/blob/master/docs/ATAK_Plugin.pdf)

**atak_bridge/PoseDescription**  
&emsp;`geometry_msgs/Pose` pose  
&emsp;`std_msgs/String` description  

**atak_bridge/PoseDescriptionStamped**  
&emsp;`std_msgs/Header` header  
&emsp;`atak_bridge/PoseDescription` pose  
&emsp;&emsp;`geometry_msgs/Pose` pose  
&emsp;&emsp;`std_msgs/String` description  
    
**atak_bridge/PoseDescriptionArray**  
&emsp;`std_msgs/Header` header  
&emsp;`atak_bridge/PoseDescription[]` pose_list  
&emsp;&emsp;`geometry_msgs/Pose` pose  
&emsp;&emsp;`std_msgs/String` description  

# DEV Notes

## Identification Clarification Needed. It is not allways clear what "uid" is.

### ATAK 
- Every event in ATAK has an **uid** (unique identifier). ATAK events require an **uid** that is a "globally unique name for this information on this event" 
- In a CoT message a sub-schema may appear in the 'detail' element. In the 'detail' element a **uid** sub element exists that "provides a place to annotate a CoT message with the unique identifier used by a particular system". 
- In CoT documentation the acronym **uid** seems to mean one of two things either **unique identifier** or **user identifier**.

### Use cases
1. An ATAK user wishes a robot to follow a route. 
    - The user draws a route on his ATAK interface using the ATAK Route Tool.
    - The user adds the robot's callsign to the Route Name field by selecting the route name and typing the robot name into the end of the existing route name.
    - The user then use the send button to send it to the atak_bridge.
    - The atak_bridge code converts it to a ROS nav_msg/Path and publishes it.
    - Questions:
        1. How to handle orientation at waypoints and end point? ATAK routes does not do orientation.
        2. How to handle altitude at waypoints and end point? ATAK routes does not do altitude.

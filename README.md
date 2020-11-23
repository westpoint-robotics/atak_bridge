# ROS to ATAK bridge
## Notes
- Based on work found at: https://github.com/pinztrek/takpak.git.
    - The python code found in the takpak directory is slightly modified versions of this code.
    - Some of the python code found outside the takpak directory is based on example code provided by takpak.  
- According to REP 105 (https://www.ros.org/reps/rep-0105.html): When defining coordinate frames with respect to a global reference like  earth align the x-axis east, y-axis north, and the z-axis up at the origin of the coordinate frame.
- According to REP 103 (https://www.ros.org/reps/rep-0103.html): By the right hand rule, the yaw component of orientation increases as the child frame rotates counter-clockwise, and for geographic poses, yaw is zero when pointing east. This requires special mention only because it differs from a traditional compass bearing, which is zero when pointing north and increments clockwise. Hardware drivers should make the appropriate transformations before publishing standard ROS messages.

- FX currently the Firewall on the server is turned off. It is blocking ICMP traffic that is required for atak to work.   


## WARNING:  
The nodes that convert between UTM and LL are not tested for navigation across UTM zones. This code is intended for development and testing of robots in a single UTM zone. It is not tested in situations where the robot crosses UTM zones.   

# ROS Nodes
## dcist_cots node     
This node is the recommend node to use for DCIST. This node listens to an ATAK server for a move-to message. If one is received the node publishes the latitude, longitude, altitude, and a time stamp.  This node also sends the current robot position to the ATAK Server.  

#### Example usage
An example of for running this node can be found in the launch directory. To run this use the below command:  
`roslaunch atak_bridge dcist_tak.launch`  

#### Parameters
- `callsign` (string, default: 'default_callsign')  
    The call sign used by this system to identify itself on TAK  
- `uid` (string, default: 'fqdn + uuid')  
    A unique identifier used by TAK to identify this system. This will be automatically set by the system. If manually set, the manual setting will override the automatically generated 'fqdn + uuid'.  
- `team_name` (string, default: 'Default Team')  
    This system's team affilation
- `team_role` (string, default: 'Default Team Role')  
    This system's role in the team
- `tak_ip` (string, default: '127.0.0.1')  
    The IP address of the server
- `tak_port` (string, default: '8088')  
    The port for an unsecure connection to the server.

#### Published Topics
- `~/atak/atak_tgt` (geometry_msgs/Vector3Stamped): Current location in latitude, longitude, and altitude that a ATAK system is requesting the robot to move to.   

#### Subscribed Topics
- `~/atak/atak_fix` (sensor_msgs/NavSatFix): Current location of the robot in the world coordinate frame. This is used to update the TAK Server on the current robot position.

## global_cords_node 
This node subscribes to the robots odometry and publishes the robot location in a world centric coordinate system using latitude and longitude. As the robot moves the latitude and longitude published is updated by adding the robot's odometry to a known start position in latitude and longitude. The start position is provided as arguments to this node. The location accuracy in latitude and longitude is dependant on the accuracy of the start location and the accuracy of the robot's odometry.

#### Parameters
- `start_latitude` (double, default: '41.39058')  
    The initial latitude when the robot's odom is all zeros.  
- `start_longitude` (double, default: '-73.95323')  
    The initial longitude when the robot's odom is all zeros.  
- `start_altitude` (double, default: '2.0')  
    The initial altitude when the robot's odom is all zeros.  
- `start_heading` (double, default: '0.0')  
    The initial heading.

#### Published Topics
- `~/atak/atak_fix` (sensor_msgs/NavSatFix): A simulated location in latitude, longitude, and altitude generated by combining the robots odometry with an initial position.   

#### Subscribed Topics
- `~/odom` (nav_msgs/Odometry): The location that the robot currently beleives it is at in meters. This is used to create the latitude and longitude.  

## goto_target node
This node listens the dcist_cots node and upon recieving a target location from it, it publishes a GotoRegionActionGoal that cause the robot to plan a path and move to that region. 

#### Parameters
- `start_latitude` (double, default: '41.39058')  
    The initial latitude when the robot's odom is all zeros.  
- `start_longitude` (double, default: '-73.95323')  
    The initial longitude when the robot's odom is all zeros.  
- `start_altitude` (double, default: '2.0')  
    The initial altitude when the robot's odom is all zeros.  
- `start_heading` (double, default: '0.0')  
    The initial heading.

#### Published Topics
- `~/goto_region/goal` (arl_nav_msgs/GotoRegionActionGoal): The region that the robot should move to. Upon publishing this topic the robot immediately moves to that location.   

#### Subscribed Topics
- `~/atak/atak_tgt` (geometry_msgs/Vector3Stamped): The location that the robot should move to in latitude and longitude.  

## fix_pub node (for testing only)
This node exists to simulate a gps on a moving robot. This node is used for testing the TAK interaction without the use of robots or simulation. It publishes a latitude and longitude at 1 HZ. Each time it publishes it adds a small offset to the current position to simulate movement.

#### Example usage
- To run this node in the CLI in support of the dcist_tak.launch it needs to be in the atak namespace for the topics to align. Use this command:
    `rosrun atak_bridge fix_pub.py __ns:=atak`
- To add this node to a launch file add this line:
    `<node name="fix_pub" pkg="atak_bridge" type="fix_pub.py" />`  

#### Published Topics
- `atak_fix` (sensor_msgs/NavSatFix): A simulated location in latitude, longitude, and altitude generated by adding an offset to the last published coordinate.   





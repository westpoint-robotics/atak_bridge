#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def driver():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('gvrbot_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = 0.5 # in M/S, negative numbers go backwards
        msg.angular.z = 0.25 # anything but zero here makes it turn
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        driver()
    except rospy.ROSInterruptException:
        pass
        
'''
export ROS_IP=192.168.0.10  #YOUR ROS MACHINE IP
export ROS_MASTER_URI=http://192.168.0.101:11311


rostopic list
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  "{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.0}}"

git clone https://github.com/westpoint-robotics/usma_gvrbot.git
'''        

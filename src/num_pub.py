#!/usr/bin/env python
# license removed for brevity
import random

import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('random_num', Int16, queue_size=10)
    rospy.init_node('num_pub', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        rnd_num = random.randint(0,50)
        rospy.loginfo(rnd_num)
        pub.publish(rnd_num)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# license removed for brevity
import rospy
#from sensor_msgs.msg import NavSatFix
from atak_bridge.msg import AtakContactList
from atak_bridge.msg import AtakContact

def talker():
    pub = rospy.Publisher('contacts', AtakContactList, queue_size=10)
    rospy.init_node('contact_sim', anonymous=True)
    contact = AtakContact()
    contact.uid = "AIDTR Gator 1" 
    contact.type = "a-f-G-E-V"
    contact.how = "m-f"
    contact.latitude = 41.393850 # Soccer field
    contact.longitude = -73.953674
    contact.altitude = 10
    contactList = []
    contactList=[contact]
    msg = AtakContactList()
    print(type(msg),"MESAGGE TYPE")
    msg.contactList = contactList

    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        #msg.header.stamp = rospy.Time.now()
        #msg.status.status = 0
        #msg.status.service = 0
        #msg.latitude = msg.latitude + 0.0005
        #msg.longitude = msg.longitude + 0.0005 
        #msg.position_covariance
        #msg.position_covariance_type
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        
        
'''
user1@rrc-prec01:~/catkin_ws$ rosmsg show atak_bridge/AtakContactList 
atak_bridge/AtakContact[] contactList
  string uid
  string type
  string how
  float64 latitude
  float64 longitude
  float64 altitude
  float64 ce
  float64 le


'''

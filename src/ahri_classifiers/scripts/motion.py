#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


pub = rospy.Publisher('moCap', String, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    hello_str = "Classified motion + " + str(data.data) + str(rospy.get_time())
    rospy.loginfo(hello_str)
    pub.publish(hello_str)

def motion():
    rospy.init_node('motionClassifer', anonymous=True)
    rospy.Subscriber("motionData", String, callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        motion()
    except rospy.ROSInterruptException:
        pass
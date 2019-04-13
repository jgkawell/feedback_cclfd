#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


pub = rospy.Publisher('/classifiers/motion', String, queue_size=10)

def callback(data):
    rospy.logwarn("MOTION: I heard %s", data.data)
    rospy.logwarn("MOTION: Classifying data...")
    pub.publish(data.data)

def motion():
    rospy.init_node('motion', anonymous=True)
    rospy.Subscriber("/sensors/mocap", String, callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        motion()
    except rospy.ROSInterruptException:
        pass
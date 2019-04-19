#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('/classifiers/face', String, queue_size=10)

def callback(data):
    rospy.logwarn("FACE: I heard %s", data.data)
    rospy.logwarn("FACE: Classifying emotion...")
    pub.publish(data.data)

def face():
    rospy.init_node('face', anonymous=True)
    rospy.Subscriber("/sensors/camera", String, callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        face()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32

pub = rospy.Publisher('/classifiers/motion', Bool, queue_size=10)

def callback(data):
    # rospy.logwarn("MOTION: I heard %s", data.data)
    # rospy.logwarn("MOTION: Classifying data...")
    pub.publish(data.data)

def motion():
    rospy.init_node('motion', anonymous=True)
    rospy.Subscriber("/sensors/mocap", Float32, callback)
    rospy.loginfo("MOTION: Starting...")
    rospy.spin()
    
if __name__ == '__main__':
    try:
        motion()
    except rospy.ROSInterruptException:
        pass
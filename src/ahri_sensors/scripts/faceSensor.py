#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def faceSensor():
    pub = rospy.Publisher('faceData', String, queue_size=10)
    rospy.init_node('faceSensor', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cameraInput = "Human Face Data  %s" % rospy.get_time()
        rospy.loginfo(cameraInput)
        pub.publish(cameraInput)
        rate.sleep()

if __name__ == '__main__':
    try:
        faceSensor()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def motionSensor():
    pub = rospy.Publisher('motionData', String, queue_size=10)
    rospy.init_node('motionSensor', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        moCapInput = "Human Motion Capture Data  %s" % rospy.get_time()
        rospy.loginfo(moCapInput)
        pub.publish(moCapInput)
        rate.sleep()

if __name__ == '__main__':
    try:
        motionSensor()
    except rospy.ROSInterruptException:
        pass
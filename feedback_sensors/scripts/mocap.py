#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def mocap():
    pub = rospy.Publisher('/sensors/mocap', String, queue_size=10)
    rospy.init_node('mocap', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        mocap_output = "MoCap time %s" % rospy.get_time()
        rospy.logwarn("MOCAP: Publishing mocap data...")
        pub.publish(mocap_output)
        rate.sleep()

if __name__ == '__main__':
    try:
        mocap()
    except rospy.ROSInterruptException:
        pass
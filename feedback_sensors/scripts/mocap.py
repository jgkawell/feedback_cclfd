#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import time
from std_msgs.msg import String

def mocap():
    rospy.init_node('mocap', anonymous=True)
    pub = rospy.Publisher('/sensors/mocap', String, queue_size=10)

    rate = rospy.Rate(0.1) # 10hz

    time.sleep(2)

    rospy.loginfo("MOCAP: Starting...")

    while not rospy.is_shutdown():
        mocap_output = "MoCap time %s" % rospy.get_time()
        # rospy.logwarn("MOCAP: Publishing mocap data...")
        pub.publish(mocap_output)
        rate.sleep()

if __name__ == '__main__':
    try:
        mocap()
    except rospy.ROSInterruptException:
        pass
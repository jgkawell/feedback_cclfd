#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import time
from std_msgs.msg import String
from lfd.items import StaticObject
def mocap():
    rospy.init_node('mocap', anonymous=True)
    pub = rospy.Publisher('/sensors/mocap', String, queue_size=10)

    rate = rospy.Rate(0.1) # 10hz

    left_hand = StaticObject(1, "left_hand", None, "world", "left_hand")


    while not rospy.is_shutdown():
        mocap_output = "MoCap time %s" % rospy.get_time()
        rospy.logwarn("MOCAP: Publishing mocap data...")
        pub.publish(mocap_output)
        print(left_hand.get_state())

if __name__ == '__main__':
    try:
        mocap()
    except rospy.ROSInterruptException:
        pass
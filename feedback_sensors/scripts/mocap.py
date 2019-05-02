#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import time
from std_msgs.msg import Float32
from lfd.items import StaticObject

def mocap():
    rospy.init_node('mocap', anonymous=True)
    pub = rospy.Publisher('/sensors/mocap', Float32, queue_size=10)
    rate = rospy.Rate(1) 
    left_hand = StaticObject(1, "left_hand", None, "world", "left_hand")
    rospy.loginfo("MOCAP: Starting...")

    while not rospy.is_shutdown():
        z = left_hand.get_state()['position'][2]
        z = round(z, 4)
        pub.publish(z)
        rate.sleep()

if __name__ == '__main__':
    try:
        mocap()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from lfd.items import StaticObject

""" This scrip simply reads in mocap data and publishes
    it as a ROS message. """


def mocap():
    # node is initialized
    rospy.init_node('mocap', anonymous=True)
    # publisher is initialized
    pub = rospy.Publisher('/sensors/mocap', Pose, queue_size=10)
    rate = rospy.Rate(1)

    # initializing the object for coordinates to be picked up from MoCAP system
    left_hand = StaticObject(1, "left_hand", None, "world", "left_hand")
    rospy.loginfo("MOCAP: Starting...")

    while not rospy.is_shutdown():
        # getting the coordinates and passing the z-axis to the classifier
        hand_pose = Pose()
        hand_pose.position = left_hand.get_state()['position']
        hand_pose.orientation = left_hand.get_state()['orientation']
        pub.publish(hand_pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        mocap()
    except rospy.ROSInterruptException:
        pass

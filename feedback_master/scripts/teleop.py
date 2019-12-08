#!/usr/bin/env python2.7
# license removed for brevity
import os
import rospy
import numpy as np

from std_msgs.msg import Bool


""" This class allows for manual triggering of classification """


class teleop():

    def __init__(self):
        rospy.init_node('teleop', anonymous=True)

        # initializing publisher
        self.pub_face = rospy.Publisher(
            '/classifiers/face', Bool, queue_size=10)
        self.pub_motion = rospy.Publisher(
            '/classifiers/motion', Bool, queue_size=10)

    def main(self):

        while not rospy.is_shutdown():
            print("TELEOP: Press [enter] to trigger" +
                  " negative classification...")
            raw_input().upper()
            print("TELEOP: Trigging negative classification!")
            self.pub_face.publish(False)
            self.pub_motion.publish(False)


if __name__ == '__main__':
    try:
        obj = teleop()
        obj.main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

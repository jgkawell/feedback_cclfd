#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import Bool
import os
import numpy as np

""" This class allows for manual triggering of classification """


class teleop():

    def __init__(self):
        rospy.init_node('teleop', anonymous=True)

        # initializing publisher
        self.pub = rospy.Publisher(
            '/classifiers/face', Bool, queue_size=10)

    def main(self):

        while not rospy.is_shutdown():
            print("TELEOP: Press [enter] to trigger" +
                  " negative classification...")
            raw_input().upper()
            print("TELEOP: Trigging negative classification!")
            self.pub.publish(False)


if __name__ == '__main__':
    try:
        obj = teleop()
        obj.main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

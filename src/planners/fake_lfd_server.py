#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np
import time

from feedback_cclfd.srv import PerformDemonstration

""" This class is simply for debugging.
    If you want to work on the system
    separate from Sawyer you can run this instead. """


class FakeLfdServer():

    def __init__(self):
        rospy.init_node('feedback_demonstration_server')

    def run(self):
        # setup service
        rospy.Service("feedback_demonstration", PerformDemonstration,
                      self.handle_feedback_demonstration)
        rospy.loginfo("FAKE LfD: Starting...")
        rospy.spin()

    def handle_feedback_demonstration(self, constraints):
        # log and then sleep to simulate demonstration being performed
        rospy.loginfo("FAKE LfD: Performing demonstration...")
        time.sleep(3)
        return True


if __name__ == '__main__':
    try:
        obj = FakeLfdServer()
        obj.run()
    except rospy.ROSInterruptException:
        pass

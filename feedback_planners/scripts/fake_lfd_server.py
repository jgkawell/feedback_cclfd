#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np

from feedback_planners.srv import PerformDemonstration


class FakeLfdServer():

    def __init__(self):
        rospy.init_node('perform_demonstration_server')

    def run(self):
        service = rospy.Service("perform_demonstration", PerformDemonstration, self.handle_perform_demonstration)
        rospy.spin()

    def handle_perform_demonstration(self, constraints):
        rospy.loginfo("FAKE LfD: Performing demonstration...")

        return True

if __name__ == '__main__':
    try:
        obj = FakeLfdServer()
        obj.run()
    except rospy.ROSInterruptException:
        pass
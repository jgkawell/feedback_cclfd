#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray

class Demonstrator():

    def __init__(self):
        rospy.init_node('demonstrator')

        # start pub/sub
        rospy.Subscriber("/planners/constraint_types", UInt8MultiArray, self.sample_demonstrations)
        self.planners_pub = rospy.Publisher("/planners/demonstrations", UInt8MultiArray, queue_size=10)
        self.lfd_pub = rospy.Publisher("/planners/perform_demonstration", String, queue_size=10)

    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0

    def sample_demonstrations(self, constraint_types):

        rospy.logwarn("DEMONSTRATOR: Sampling demonstrations...")

        self.lfd_pub.publish("Do a demonstration!")

        array = []
        array_msg = UInt8MultiArray(data=array)
        self.planners_pub.publish(array_msg)


if __name__ == '__main__':
    try:
        obj = Demonstrator()
        obj.run()
    except rospy.ROSInterruptException:
        pass
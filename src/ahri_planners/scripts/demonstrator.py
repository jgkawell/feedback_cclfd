#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray

class Demonstrator():

    def __init__(self):
        rospy.init_node('demonstrator')

        # start pub/sub
        rospy.Subscriber("/planners/constraint_types", UInt8MultiArray, self.sample_demonstrations)
        self.demonstrations_pub = rospy.Publisher("/planners/demonstrations", UInt8MultiArray, queue_size=10)
        

    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0

    def sample_demonstrations(self, constraint_types):

        rospy.logwarn("DEMONSTRATOR: Sampling demonstrations...")

        array = np.zeros(1)
        self.demonstrations_pub.publish(1, array)


if __name__ == '__main__':
    try:
        obj = Demonstrator()
        obj.run()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray

class PolicyAugmentation():

    def __init__(self):
        rospy.init_node('policy_augmentation')

        # start pub/sub
        rospy.Subscriber("/planners/constraints", UInt8MultiArray, self.extract_features)
        self.model_pub = rospy.Publisher("/planners/update_model", String, queue_size=10)

    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0

    def extract_features(self, demonstrations):

        rospy.logwarn("POLICY AUGMENTATION: Augmenting policy...")
        self.model_pub.publish("Update model!")


if __name__ == '__main__':
    try:
        obj = PolicyAugmentation()
        obj.run()
    except rospy.ROSInterruptException:
        pass
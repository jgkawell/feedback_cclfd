#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np

from std_msgs.msg import UInt8MultiArray

class FeatureExtraction():

    def __init__(self):
        rospy.init_node('feature_extraction')

        # start pub/sub
        rospy.Subscriber("/planners/demonstrations", UInt8MultiArray, self.extract_features)
        self.constraints_pub = rospy.Publisher("/planners/constraints", UInt8MultiArray, queue_size=10)
        

    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0

    def extract_features(self, demonstrations):

        rospy.logwarn("FEATURE EXTRACTION: Extracting features...")

        array = []
        array_msg = UInt8MultiArray(data=array)
        self.constraints_pub.publish(array_msg)


if __name__ == '__main__':
    try:
        obj = FeatureExtraction()
        obj.run()
    except rospy.ROSInterruptException:
        pass
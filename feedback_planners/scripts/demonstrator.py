#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import json

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray

class Demonstrator():

    def __init__(self):
        rospy.init_node('demonstrator')

        # start pub/sub
        rospy.Subscriber("/planners/constraint_types", UInt8MultiArray, self.sample_demonstrations)
        self.planners_pub = rospy.Publisher("/planners/demonstrations", String, queue_size=10)

        # set up client for demonstration service
        rospy.wait_for_service("perform_demonstration")
        try:
            self.perform_demonstration = rospy.ServiceProxy("perform_demonstration", PerformDemonstration)
        except rospy.ServiceException, e:
            print("Service call failed: %s", e)

        # set up client for feedback service
        rospy.wait_for_service("request_feedback")
        try:
            self.request_feedback = rospy.ServiceProxy("request_feedback", RequestFeedback)
        except rospy.ServiceException, e:
            print("Service call failed: %s", e)

    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0

    def sample_demonstrations(self, constraint_types):

        rospy.logwarn("DEMONSTRATOR: Sampling demonstrations...")

        results = dict()
        for constraint in constraint_types.data:
            # perform a single demonstration
            temp_array = [constraint]
            self.perform_demonstration(temp_array)

            # request feedback about demonstration from user
            response = self.request_feedback()

            results[temp_array] = response

        encoded_data_string = json.dumps(results)
        self.planners_pub.publish(encoded_data_string)


if __name__ == '__main__':
    try:
        obj = Demonstrator()
        obj.run()
    except rospy.ROSInterruptException:
        pass
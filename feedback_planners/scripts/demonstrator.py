#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np
import json

from std_msgs.msg import String
from std_msgs.msg import Bool
from rospy.numpy_msg import numpy_msg

from feedback_planners.srv import RequestFeedback
from feedback_planners.srv import PerformDemonstration
from feedback_planners.msg import ConstraintTypes

class Demonstrator():

    def __init__(self):
        rospy.init_node('demonstrator')

        # start pub/sub
        rospy.Subscriber("/planners/constraint_types", numpy_msg(ConstraintTypes), self.sample_demonstrations)
        self.demos_pub = rospy.Publisher("/planners/demonstrations", String, queue_size=10)

        # set up client for demonstration service
        rospy.wait_for_service("perform_demonstration")
        try:
            self.perform_demonstration = rospy.ServiceProxy("perform_demonstration", PerformDemonstration)
            rospy.logwarn("Service setup succeeded (perform_demonstration)")
        except rospy.ServiceException:
            rospy.logwarn("Service setup failed (perform_demonstration)")

        # set up client for feedback service
        rospy.wait_for_service("request_feedback")
        try:
            self.request_feedback = rospy.ServiceProxy("request_feedback", RequestFeedback)
            rospy.logwarn("Service setup succeeded (request_feedback)")
        except rospy.ServiceException:
            rospy.logwarn("Service setup failed (request_feedback)")

    def run(self):
        rospy.spin()

    def sample_demonstrations(self, constraint_types):
        num_demos = 5


        rospy.logwarn("DEMONSTRATOR: Sampling demonstrations...")
        
        cur_type = constraint_types.data


        results = dict()
        for i in range(0, num_demos):
            # perform a single demonstration
            temp_array = [cur_type]
            finished = self.perform_demonstration(temp_array)

            if finished:
                # request feedback about demonstration from user
                response = self.request_feedback(True)
                key = (cur_type, i)

                if response:
                    results[str(key)] = 1
                else:
                    results[str(key)] = 0

        encoded_data_string = json.dumps(results)
        self.demos_pub.publish(encoded_data_string)


if __name__ == '__main__':
    try:
        obj = Demonstrator()
        obj.run()
    except rospy.ROSInterruptException:
        pass
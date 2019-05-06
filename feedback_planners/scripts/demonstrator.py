#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np
import json
import time

from std_msgs.msg import String
from std_msgs.msg import Bool
from rospy.numpy_msg import numpy_msg

from feedback_planners.srv import RequestFeedback
from feedback_planners.srv import PerformDemonstration
from feedback_planners.msg import ConstraintTypes

class Demonstrator():

    def __init__(self):
        rospy.init_node('demonstrator')
        self.finished_first_demo = False

        # start pub/sub
        rospy.Subscriber("/planners/constraint_types", numpy_msg(ConstraintTypes), self.sample_demonstrations)
        self.demos_pub = rospy.Publisher("/planners/demonstrations", String, queue_size=10)

        # set up client for demonstration service
        rospy.wait_for_service("perform_demonstration")
        try:
            self.perform_demonstration = rospy.ServiceProxy("perform_demonstration", PerformDemonstration)
            rospy.loginfo("Service setup succeeded (perform_demonstration)")
        except rospy.ServiceException:
            rospy.logwarn("Service setup failed (perform_demonstration)")

        # set up client for feedback service
        rospy.wait_for_service("request_feedback")
        try:
            self.request_feedback = rospy.ServiceProxy("request_feedback", RequestFeedback)
            rospy.loginfo("Service setup succeeded (request_feedback)")
        except rospy.ServiceException:
            rospy.logwarn("Service setup failed (request_feedback)")

        rospy.loginfo("DEMONSTRATOR: Starting...")

    def run(self):
        # perform a bad demo to start
        finished = self.perform_demonstration(0) # 0 = negative

        if finished.response:
            self.finished_first_demo = True

        rospy.spin()

    def sample_demonstrations(self, constraint_types):
        # run until complete
        while True:
            # don't perform alternative demos until first is finished
            if self.finished_first_demo:
                num_demos = 2
                rospy.loginfo("DEMONSTRATOR: Sampling demonstrations...")
                cur_type = constraint_types.data
                results = dict()
                for i in range(0, num_demos):
                    # perform a single demonstration
                    temp_array = i
                    finished = self.perform_demonstration(temp_array)
                    if finished.response:
                        # request feedback about demonstration from user
                        msg = self.request_feedback(True)
                        key = i
                        if msg.response:
                            rospy.loginfo("DEMONSTRATOR: Response was POSITIVE!")
                            results[key] = 1
                        else:
                            rospy.loginfo("DEMONSTRATOR: Response was NEGATIVE")
                            results[key] = 0

                # save feedback results
                rospy.loginfo("DEMONSTRATOR: Saving feedback...")
                encoded_data_string = json.dumps(results)
                self.demos_pub.publish(encoded_data_string)

                # demonstrate what has been learned
                rospy.loginfo("DEMONSTRATOR: Showing what has been learned...")
                for key, value in results.items():
                    if value:
                        temp_array = key
                        self.perform_demonstration(temp_array)
                        break
                break
            else:
                # wait a second
                rospy.loginfo("DEMONSTRATOR: Waiting for first demo to be finished...")
                time.sleep(1)

        rospy.loginfo("FINISHED!!!")


if __name__ == '__main__':
    try:
        obj = Demonstrator()
        obj.run()
    except rospy.ROSInterruptException:
        pass
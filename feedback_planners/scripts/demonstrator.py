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
from feedback_planners.srv import TTS, TTSResponse

""" This class is responsible for sampling constraints and
    demonstrating them to the user for feedback. """


class Demonstrator():

    def __init__(self):
        rospy.init_node('demonstrator')
        self.finished_first_demo = False

        # start pub/sub
        rospy.Subscriber("/planners/constraint_types",
                         numpy_msg(ConstraintTypes),
                         self.sample_demonstrations)
        self.demos_pub = rospy.Publisher(
            "/planners/demonstrations", String, queue_size=10)

        # set up client for demonstration service
        rospy.wait_for_service("feedback_demonstration")
        try:
            self.feedback_demonstration = rospy.ServiceProxy(
                "feedback_demonstration", PerformDemonstration)
        except rospy.ServiceException:
            rospy.logwarn("Service setup failed (feedback_demonstration)")

        # set up client for feedback service
        rospy.wait_for_service("request_feedback")
        try:
            self.request_feedback = rospy.ServiceProxy(
                "request_feedback", RequestFeedback)
        except rospy.ServiceException:
            rospy.logwarn("Service setup failed (request_feedback)")

        # Set up client for NLP TTS service
        rospy.wait_for_service("/nlp/tts")
        try:
            self.tts_server = rospy.ServiceProxy(
                "/nlp/tts", TTS)
        except rospy.ServiceException:
            rospy.logerr("Service setup failed (/nlp/tts)")

        rospy.loginfo("DEMONSTRATOR: Starting...")

    def run(self):
        # perform a bad demo to start
        rospy.loginfo("DEMONSTRATOR: Starting first skill execution...")
        self.tts_server("I am going to hand you the mug.")
        finished = self.feedback_demonstration(0)  # 0 = negative

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
                    constraint = i
                    self.tts_server("I am going to try the skill again.")
                    finished = self.feedback_demonstration(constraint)
                    if finished.response:
                        # request feedback about demonstration from user
                        feedback_type = constraint == 1
                        msg = self.request_feedback(feedback_type)
                        key = i
                        if msg.response:
                            rospy.loginfo(
                                "DEMONSTRATOR: Response was POSITIVE!")
                            results[key] = 1
                        else:
                            rospy.loginfo(
                                "DEMONSTRATOR: Response was NEGATIVE")
                            results[key] = 0

                # save feedback results
                rospy.loginfo("DEMONSTRATOR: Saving feedback...")
                encoded_data_string = json.dumps(results)
                self.demos_pub.publish(encoded_data_string)

                # demonstrate what has been learned
                rospy.loginfo("DEMONSTRATOR: Showing what has been learned...")
                self.tts_server("Let me show you what I have learned.")
                for key, value in results.items():
                    if value:
                        constraint = key
                        self.feedback_demonstration(constraint)
                        break
                break
            else:
                # wait a second
                rospy.loginfo(
                    "DEMONSTRATOR: Waiting for first demo to be finished...")
                time.sleep(1)

        self.tts_server("Thank you for helping me learn!")
        rospy.loginfo("FINISHED!!!")


if __name__ == '__main__':
    try:
        obj = Demonstrator()
        obj.run()
    except rospy.ROSInterruptException:
        pass

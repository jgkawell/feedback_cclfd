#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np

from feedback_planners.srv import RequestFeedback

""" This class requests simple postive/negative feedback from the user after each demonstration. """
class RequestFeedbackServer():

    def __init__(self):
        rospy.init_node('request_feedback_server')

    def run(self):
        # setup service
        rospy.Service("request_feedback", RequestFeedback, self.handle_request_feedback)
        rospy.spin()

    def handle_request_feedback(self, temp):
        rospy.loginfo("REQUEST FEEDBACK: Requesting feedback...")


        # TODO: Replace this with a speech-to-text listener
        # request feedback from user (this is done with keyboard input for now)
        print("Good or bad demonstration? (T/F)")
        response = raw_input().upper()
        print("You responded: %s", response)

        # TODO: Replace this with a positive/negative classifier
        # convert string feedback to boolean
        feedback = False
        if response == "T":
            feedback = True

        return feedback

if __name__ == '__main__':
    try:
        obj = RequestFeedbackServer()
        obj.run()
    except rospy.ROSInterruptException:
        pass
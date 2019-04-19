#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np

from feedback_planners.srv import RequestFeedback


class RequestFeedbackServer():

    def __init__(self):
        rospy.init_node('request_feedback_server')

    def run(self):
        service = rospy.Service("request_feedback", RequestFeedback, self.handle_request_feedback)
        rospy.spin()

    def handle_request_feedback(self, temp):
        rospy.logwarn("REQUEST FEEDBACK: Requesting feedback...")

        print("Good or bad demonstration? (T/F)")
        response = raw_input().upper()
        print("You responded: %s", response)

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
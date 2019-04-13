#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np

class RequestFeedback():

    def __init__(self):
        rospy.init_node('request_feedback')

    def run(self):
        service = rospy.Service("request_feedback", RequestFeedback, self.handle_request_feedback)

    def handle_request_feedback(self):
        rospy.logwarn("REQUEST FEEDBACK: Requesting feedback...")
        return True

if __name__ == '__main__':
    try:
        obj = RequestFeedback()
        obj.run()
    except rospy.ROSInterruptException:
        pass
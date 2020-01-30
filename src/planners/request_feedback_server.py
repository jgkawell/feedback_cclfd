#!/usr/bin/env python2.7
# license removed for brevity

import re
import time
import rospy
import numpy as np

from feedback_cclfd.srv import RequestFeedback
from cairo_nlp.srv import STT, STTResponse, TTS, TTSResponse

""" This class requests simple postive/negative feedback
    from the user after each demonstration. """


class RequestFeedbackServer():

    def __init__(self):
        rospy.init_node('request_feedback_server')

        # Get filepath of demo voice data from rosparam
        self.demo_filepath = rospy.get_param("DEMO_VOICE_FILEPATH")

        # Set up server
        rospy.Service("request_feedback", RequestFeedback,
                      self.handle_request_feedback)

        # Set up client for NLP TTS service
        rospy.wait_for_service("/nlp/google/tts")
        try:
            self.tts_server = rospy.ServiceProxy(
                "/nlp/google/tts", TTS)
        except rospy.ServiceException:
            rospy.logerr("Service setup failed (/nlp/google/tts)")

        # Set up client for NLP STT service
        rospy.wait_for_service("/nlp/google/stt")
        try:
            self.stt_server = rospy.ServiceProxy(
                "/nlp/google/stt", STT)
        except rospy.ServiceException:
            rospy.logerr("Service setup failed (/nlp/google/stt)")

    def main(self):
        rospy.loginfo("REQUEST FEEDBACK: Starting...")
        rospy.spin()

    def handle_request_feedback(self, feedback_type):
        rospy.loginfo("REQUEST FEEDBACK: Requesting feedback...")

        # Request feedback from user using NLP engine
        if not self.tts_server("Was that a good demonstration?"):
            rospy.logerr("REQUEST FEEDBACK: TTS failed!")

        # Wait for response
        time.sleep(5)

        # Create text response using NLP engine
        if feedback_type.input:
            response = self.stt_server(
                self.demo_filepath + "/good-demonstration.wav").output
        else:
            response = self.stt_server(
                self.demo_filepath + "/bad-demonstration.wav").output

        # Wait for response
        time.sleep(5)

        # Check if response was positive or negative
        if 'yes' in response.lower():
            rospy.loginfo("REQUEST FEEDBACK: Positive response")
            feedback = True
        else:
            rospy.loginfo("REQUEST FEEDBACK: Negative response")
            feedback = False

        return feedback


if __name__ == '__main__':
    try:
        obj = RequestFeedbackServer()
        obj.main()
    except rospy.ROSInterruptException:
        pass

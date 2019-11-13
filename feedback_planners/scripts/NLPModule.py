#!/usr/bin/env python2.7
# license removed for brevity

import rospy

""" This NLP command module can be used to invoke NLP related services/command such as:
    1. text to speech
    2. Speech to text """

class NLPModule():

    #TODO: Convert this to command node
    def __init__(self):
        rospy.init_node('nlp_module')

    def textToSpeech(self, text_message):
        """
        This method uses tts api to convert text to speech
        :param text_message: string which is needed to be converted to speech
        :return: Audio file

        """
        pass


    def speechToText(self, audio_message):
        """
        This method uses stt api to convert speech to text
        :param audio_message:
        :return:
        """
        pass


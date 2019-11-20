#!/usr/bin/env python2.7
# license removed for brevity

import rospy

from feedback_planners.srv import STT, TTS

""" This NLP command module can be used to invoke NLP related services/command such as:
    1. text to speech
    2. Speech to text """

class NLPModule():

    #TODO: Convert this to command node
    def __init__(self):
        rospy.init_node('nlp_module')

    def run(self):
        # setup services
        rospy.Service("/nlp/tts", TTS, self.textToSpeech)
        rospy.Service("/nlp/stt", STT, self.speechToText)
        rospy.loginfo("FAKE LfD: Starting...")
        rospy.spin()


    def textToSpeech(self, text_message):
        """
        This method uses tts api to convert text to speech
        :param text_message: string which is needed to be converted to speech
        :return success: whether or not the TTS succeeded

        """

        # TODO: Use an API to generate audio from text

        pass


    def speechToText(self, audio_message):
        """
        This method uses stt api to convert speech to text
        :param input: file path or trigger to record
        :return output: string created from audio
        """

        # TODO: Use an API to generate text from speech

        pass


if __name__ == '__main__':
    try:
        obj = NLPModule()
        obj.run()
    except rospy.ROSInterruptException:
        pass
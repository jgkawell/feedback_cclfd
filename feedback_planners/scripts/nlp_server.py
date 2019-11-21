#!/usr/bin/env python2.7
# license removed for brevity

import rospy

from google.cloud import texttospeech
from feedback_planners.srv import STT, TTS
from random import randint

""" This NLP command module can be used to invoke NLP such as:
    1. text to speech
    2. Speech to text """


class NLPServer():

    def __init__(self):
        rospy.init_node("nlp_server")

    def run(self):
        """
        This method initializes the ROS services

        """
        rospy.Service("/nlp/tts", TTS, self.textToSpeech)
        rospy.Service("/nlp/stt", STT, self.speechToText)
        rospy.loginfo("NLP Server: Starting...")
        rospy.spin()

    def textToSpeech(self, text_message):
        """
        This method uses tts api to convert text to speech
        :param text_message: string which is needed to be converted to speech
        :return success: whether or not the TTS succeeded

        """

        # Initialize success flag and log info
        success = True
        rospy.loginfo("NLP Server: Attempting to create speech from text: %s",
                      text_message.input.data)

        # Instantiates a client
        client = texttospeech.TextToSpeechClient()

        # Set the text input to be synthesized
        synthesis_input = texttospeech.types.SynthesisInput(
            text=text_message.input.data)

        # Build the voice request
        voice = texttospeech.types.VoiceSelectionParams(
            language_code="en-US",
            ssml_gender=texttospeech.enums.SsmlVoiceGender.FEMALE)

        # Select the type of audio file
        audio_config = texttospeech.types.AudioConfig(
            audio_encoding=texttospeech.enums.AudioEncoding.MP3)

        try:
            # Perform the text-to-speech request on the text input with the
            # selected voice parameters and audio file type
            response = client.synthesize_speech(
                synthesis_input, voice, audio_config)

            # Generate random file name
            temp_num = randint(100000, 999999)
            file_name = "../out/output" + str(temp_num) + ".mp3"

            # The response"s audio_content is binary.
            with open(file_name, "wb") as out:
                # Write the response to the output file.
                out.write(response.audio_content)
                rospy.loginfo("Audio content written to file: %s", file_name)
        except Exception as ex:
            rospy.logerr("NLP Server: %s", str(ex))
            success = False

        return success

    def speechToText(self, audio_message):
        """
        This method uses stt api to convert speech to text
        :param input: file path or trigger to record
        :return output: string created from audio
        """

        # TODO: Use an API to generate text from speech

        pass


if __name__ == "__main__":
    try:
        obj = NLPServer()
        obj.run()
    except rospy.ROSInterruptException:
        pass

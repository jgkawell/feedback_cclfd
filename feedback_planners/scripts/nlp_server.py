#!/usr/bin/env python2.7
# license removed for brevity

import rospy
import io
import os
import errno

# Google Cloud API
from google.cloud import texttospeech
from google.cloud import speech_v1
from google.cloud.speech_v1 import enums

# Other imports
from std_msgs.msg import String
from feedback_planners.srv import STT, STTResponse, TTS, TTSResponse
from random import randint

""" This NLP command module can be used to invoke NLP such as:
    1. text to speech
    2. Speech to text """


class NLPServer():

    def __init__(self):
        rospy.init_node("nlp_server")
        # Create a repeater for user input
        self.pub = rospy.Publisher( "/viz/user_feedback" , String , queue_size = 100 )

    def run(self):
        """
        This method initializes the ROS services

        """
        rospy.Service("/nlp/tts", TTS, self.textToSpeech)
        rospy.Service("/nlp/stt", STT, self.speechToText)
        rospy.loginfo("NLP Server: Starting...")

        rospy.spin()

    def textToSpeech(self, tts_srv):
        """
        This method uses tts api to convert text to speech
        https://cloud.google.com/text-to-speech/docs/quickstart-client-libraries#client-libraries-install-python
        :param text_message: string which is needed to be converted to speech
        :return success: whether or not the TTS succeeded

        """

        # Initialize success flag and log info
        success = True
        rospy.loginfo("NLP Server TTS: Computing speech from text: %s",
                      tts_srv.input)

        # Instantiates a client
        client = texttospeech.TextToSpeechClient()

        # Set the text input to be synthesized
        synthesis_input = texttospeech.types.SynthesisInput(
            text=tts_srv.input)

        # Build the voice request
        voice = texttospeech.types.VoiceSelectionParams(
            language_code="en-US",
            ssml_gender=texttospeech.enums.SsmlVoiceGender.FEMALE)

        # Select the type of audio file
        audio_config = texttospeech.types.AudioConfig(
            audio_encoding=texttospeech.enums.AudioEncoding.LINEAR16)

        try:
            # Perform the text-to-speech request on the text input with the
            # selected voice parameters and audio file type
            response = client.synthesize_speech(
                synthesis_input, voice, audio_config)

            # Generate random file name name so nothing is overwritten
            cur_file = os.path.dirname(os.path.abspath(__file__))
            temp_num = randint(100000, 999999)
            file_name = cur_file + "/../out/output" + str(temp_num) + ".wav"

            # Make sure file exists
            if not os.path.exists(os.path.dirname(file_name)):
                try:
                    os.makedirs(os.path.dirname(file_name))
                except OSError as exc:  # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise

            # The response"s audio_content is binary.
            with open(file_name, "wb") as out:
                # Write the response to the output file.
                out.write(response.audio_content)
                rospy.loginfo("Audio content written to file: %s", file_name)

        except Exception as ex:
            rospy.logerr("NLP Server TTS: %s", str(ex))
            success = False

        return success

    def speechToText(self, stt_srv):
        """
        This method uses stt api to convert speech to text
        https://cloud.google.com/speech-to-text/docs/quickstart-client-libraries
        :param input: file path or trigger to record
        :return output: string created from audio
        """

        response_text = ""
        rospy.loginfo("NLP Server STT: Computing text from speech: %s",
                      stt_srv.input)
        try:
            # Create client object
            client = speech_v1.SpeechClient()

            # Configure API request
            config = {
                "language_code": "en-US",
                "sample_rate_hertz": 24000,
                "encoding": enums.RecognitionConfig.AudioEncoding.LINEAR16,
            }

            # Build audio data from file
            with io.open(stt_srv.input, "rb") as f:
                content = f.read()
            audio = {"content": content}

            # Attempt to read text from audio file
            response = client.recognize(config, audio)
            for result in response.results:
                cur_result = u"{}".format(result.alternatives[0].transcript)
                rospy.loginfo("NLP Server STT: Transcript: %s", cur_result)
                response_text += cur_result.encode('ascii', 'ignore')

        except Exception as ex:
            rospy.logerr("NLP Server STT: %s", str(ex))

        # Return the text
        msg = String()
        msg.data = response_text
        self.pub.publish( msg )
        return response_text


if __name__ == "__main__":
    try:
        obj = NLPServer()
        obj.run()
    except rospy.ROSInterruptException:
        pass

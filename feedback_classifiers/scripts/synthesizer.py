#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

""" This class synthesizes the various classifications of human affect/motion to see if the behavior is negative """
class synthesizer():

    def __init__(self):
        self.face_label = True
        self.motion_label = True

        # initialize pub/sub
        rospy.init_node('synthesizer', anonymous=True)
        rospy.Subscriber("/classifiers/face", Bool, self.callback_face)
        rospy.Subscriber("/classifiers/motion", Bool, self.callback_motion)
        self.synthesis_pub = rospy.Publisher('/classifiers/synthesis', Bool, queue_size=10)
        
        rospy.loginfo("SYNTHESIZER: Starting...")

    def main(self):
        # loop checking the labels sent in by the other classifiers
        triggered = False
        while(not rospy.is_shutdown()):

            # TODO: Replace this with a weighted sum based on confidence levels
            # if the face is negative or the motion is negative, trigger skill repair
            if (not face_label or not motion_label) and not triggered:
                rospy.loginfo("SYNTHESIZER: Recognized a negative response!")
                self.synthesis_pub.publish(True)
                triggered = True

    def callback_face(self, data):
        global face_label
        face_label = data.data

    def callback_motion(self, data):
        global motion_label
        motion_label = data.data

if __name__ == '__main__':
    try:
        obj = synthesizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from feedback_classifiers.msg import Classification


""" This class synthesizes the various classifications of human
    affect/motion to see if the behavior is negative """


class synthesizer():

    def __init__(self):
        self.face = True
        self.motion = True

        # initialize pub/sub
        rospy.init_node('synthesizer', anonymous=True)
        rospy.Subscriber("/classifiers/face", Bool, self.callback_face)
        rospy.Subscriber("/classifiers/motion", Bool, self.callback_motion)
        self.synthesis_pub = rospy.Publisher(
            '/classifiers/synthesis', Classification, queue_size=10)

        rospy.loginfo("SYNTHESIZER: Starting...")

    def main(self):
        # loop checking the labels sent in by the other classifiers
        triggered = False
        msg = Classification()
        while not rospy.is_shutdown():

            # TODO: Replace this with a weighted sum based on confidence levels
            # if the face is negative or the motion is negative, trigger repair
            if (not self.face or not self.motion) and not triggered:
                rospy.loginfo("SYNTHESIZER: Recognized a negative response!")
                msg.timestamp = rospy.Time.now()
                msg.classification = False
                self.synthesis_pub.publish(msg)
                triggered = True

    def callback_face(self, msg):
        self.face = msg.data

    def callback_motion(self, msg):
        self.motion = msg.data


if __name__ == '__main__':
    try:
        obj = synthesizer()
        obj.main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

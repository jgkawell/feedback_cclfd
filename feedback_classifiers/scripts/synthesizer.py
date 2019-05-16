#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

face_label = True
motion_label = True

def callback_face(data):
    global face_label
    face_label = data.data

def callback_motion(data):
    global motion_label
    motion_label = data.data

def synthesizer():
    global face_label
    global motion_label

    # initialize pub/sub
    rospy.init_node('synthesizer', anonymous=True)
    rospy.Subscriber("/classifiers/face", Bool, callback_face)
    rospy.Subscriber("/classifiers/motion", Bool, callback_motion)
    synthesis_pub = rospy.Publisher('/classifiers/synthesis', Bool, queue_size=10)
    
    rospy.loginfo("SYNTHESIZER: Starting...")

    triggered = False
    while(not rospy.is_shutdown()):

        # TODO: Replace this with a weighted sum based on confidence levels
        # if the face is negative or the motion is negative, trigger skill repair
        if (not face_label or not motion_label) and not triggered:
            rospy.loginfo("SYNTHESIZER: Recognized a negative response!")
            synthesis_pub.publish(True)
            triggered = True

if __name__ == '__main__':
    try:
        synthesizer()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

have_face = False
have_motion = False

def callback_face(data):
    global have_face
    # rospy.logwarn("SYNTHESIZER: I heard %s", data.data)
    have_face = True

def callback_motion(data):
    global have_motion
    # rospy.logwarn("SYNTHESIZER: I heard %s", data.data)
    have_motion = True

def synthesizer():
    global have_face
    global have_motion

    rospy.init_node('synthesizer', anonymous=True)
    rospy.Subscriber("/classifiers/face", String, callback_face)
    rospy.Subscriber("/classifiers/motion", String, callback_motion)
    synthesis_pub = rospy.Publisher('/classifiers/synthesis', Bool, queue_size=10)
    
    rospy.loginfo("SYNTHESIZER: Starting...")

    triggered = False
    while(not rospy.is_shutdown()):
        if have_face and have_motion and not triggered:
            rospy.loginfo("SYNTHESIZER: Synthesizing classifications...")
            have_face = False
            have_motion = False

            synthesis_pub.publish(True)
            triggered = True

if __name__ == '__main__':
    try:
        synthesizer()
    except rospy.ROSInterruptException:
        pass
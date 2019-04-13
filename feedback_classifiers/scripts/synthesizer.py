#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

have_face = False
have_motion = False

def callback_face(data):
    global have_face
    rospy.logwarn("SYNTHESIZER: I heard %s", data.data)
    have_face = True

def callback_motion(data):
    global have_motion
    rospy.logwarn("SYNTHESIZER: I heard %s", data.data)
    have_motion = True

def synthesizer():
    global have_face
    global have_motion

    rospy.init_node('synthesizer', anonymous=True)
    rospy.Subscriber("/classifiers/face", String, callback_face)
    rospy.Subscriber("/classifiers/motion", String, callback_motion)
    pub = rospy.Publisher('/classifiers/synthesis', Bool, queue_size=10)

    
    while(not rospy.is_shutdown()):
        if have_face and have_motion:
            rospy.logwarn("SYNTHESIZER: Synthesizing classifications...")
            have_face = False
            have_motion = False

            pub.publish(True)

if __name__ == '__main__':
    try:
        synthesizer()
    except rospy.ROSInterruptException:
        pass
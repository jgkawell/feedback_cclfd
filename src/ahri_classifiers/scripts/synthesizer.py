#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def callbackemotion(data):
    rospy.loginfo(rospy.get_caller_id() + "Emotion data heard %s", data.data)

def callbackmotion(data):
    rospy.loginfo(rospy.get_caller_id() + "Motion data heard %s", data.data)

def synthesizer():
    rospy.init_node('emotionSynthesizer', anonymous=True)
    rospy.Subscriber("facialEmotion", String, callbackemotion)
    rospy.Subscriber("moCap", String, callbackmotion)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        synthesizer()
    except rospy.ROSInterruptException:
        pass
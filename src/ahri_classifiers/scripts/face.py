#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

pub = rospy.Publisher('facialEmotion', String, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    classifiedEmotion = "Classified emotion + " + str(data.data) + str(rospy.get_time())
    rospy.loginfo(classifiedEmotion)
    pub.publish(classifiedEmotion)

def face():
    rospy.init_node('faceClassifer', anonymous=True)
    rospy.Subscriber("faceData", String, callback)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        face()
    except rospy.ROSInterruptException:
        pass
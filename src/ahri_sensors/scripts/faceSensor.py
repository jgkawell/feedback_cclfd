import rospy
from std_msgs.msg import String

def faceSensor():
    pub = rospy.Publisher('faceData', String, queue_size=10)
    rospy.init_node('faceSensor', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        faceInput = "Human Face Data  %s" % rospy.get_time()
        rospy.loginfo(faceInput)
        pub.publish(faceInput)
        rate.sleep()

if __name__ == '__main__':
    try:
        faceSensor()
    except rospy.ROSInterruptException:
        pass
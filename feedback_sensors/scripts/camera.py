#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import time
from std_msgs.msg import String

def camera():
    rospy.init_node('camera', anonymous=True)

    pub = rospy.Publisher('/sensors/camera', String, queue_size=10)
    rate = rospy.Rate(0.1) # 10hz

    time.sleep(2)

    while not rospy.is_shutdown():
        camera_output = "Camera time %s" % rospy.get_time()
        rospy.logwarn("CAMERA: Publishing camera data...")
        pub.publish(camera_output)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
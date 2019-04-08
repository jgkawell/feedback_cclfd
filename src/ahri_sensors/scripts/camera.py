#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def camera():
    pub = rospy.Publisher('/sensors/camera', String, queue_size=10)
    rospy.init_node('camera', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
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
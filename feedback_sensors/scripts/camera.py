#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import time
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

""" This script simply reads in camera data and
    publishes it as a ROS message. """


def camera():
    # node initialized
    rospy.init_node('camera', anonymous=True)

    # object for video input
    cap = cv2.VideoCapture(0)
    # object to convert cv2 image to ROS image message
    bridge = CvBridge()
    # publisher initialized
    pub = rospy.Publisher('/sensors/camera', Image, queue_size=10)

    rospy.loginfo("CAMERA: Starting...")
    SHOW_CAMERA = rospy.get_param("SHOW_CAMERA")

    while not rospy.is_shutdown():
        while(True):
            # Capture frame-by-frame
            ret, frame = cap.read()

            # display the resulting frame
            if SHOW_CAMERA:
                cv2.imshow('input-camera.py', frame)
            try:
                # publishing image message
                pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
            except Exception as e:
                rospy.logerr(str(e))
                # quitting if 'q'/'Q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # when everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass

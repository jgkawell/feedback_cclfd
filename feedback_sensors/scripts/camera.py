#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import time
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def camera():
    rospy.init_node('camera', anonymous=True)
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    pub = rospy.Publisher('/sensors/camera', Image,queue_size=10)

    while not rospy.is_shutdown():
        camera_output = "Camera time %s" % rospy.get_time()
        SHOW_CAMERA = rospy.get_param("SHOW_CAMERA")
        # SHOW_CAMERA = True
        while(True):
            # Capture frame-by-frame
            ret, frame = cap.read()
    
            # Display the resulting frame
            if SHOW_CAMERA:
                cv2.imshow('input-camera.py',frame)
            try:
                pub.publish(bridge.cv2_to_imgmsg(frame,"bgr8"))
                rospy.logwarn("CAMERA: Image Published...")
            except e:
                print str(e)    
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass

        
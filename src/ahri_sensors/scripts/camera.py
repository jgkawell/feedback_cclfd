#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import cv2
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera():

    # NOTE: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

    def __init__(self):
        rospy.init_node('camera')
        self.bridge = CvBridge()

        # start pub/sub
        self.image_pub = rospy.Publisher("/camera/image", Image, queue_size=10)
        

    def run(self):
        while(not rospy.is_shutdown()):
            rospy.logwarn("CAMERA: Reading in camera data...")

            ros_image = Image()
            self.image_pub.publish(ros_image)

            time.sleep(1)

        

if __name__ == '__main__':
    try:
        obj = Camera()
        obj.run()
    except rospy.ROSInterruptException:
        pass
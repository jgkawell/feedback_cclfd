#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class face():

    def __init__(self):
        rospy.init_node('face', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/sensors/camera", Image, self.callback)
        self.image_pub = rospy.Publisher('/classifiers/face', String, queue_size=10)

    
    def run(self):
        service = rospy.Service("perform_demonstration", PerformDemonstration, self.handle_perform_demonstration)
        rospy.spin()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow('output-face',cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    # rospy.logwarn("FACE: I heard %s", data.data)
    # rospy.logwarn("FACE: Classifying emotion...")
    # pub.publish(data.data)
    
if __name__ == '__main__':
    try:
        obj = face()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



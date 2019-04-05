#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import time

from std_msgs.msg import String

class MoCap():

    def __init__(self):
        rospy.init_node('mocap')

        # start pub/sub
        self.image_pub = rospy.Publisher("/mocap/data", String, queue_size=10)
        

    def run(self):
        while(not rospy.is_shutdown()):
            rospy.logwarn("MOCAP: Reading in motion capture data...")

            mocap_msg = "Hello World"
            self.image_pub.publish(mocap_msg)

            time.sleep(1)        

if __name__ == '__main__':
    try:
        obj = MoCap()
        obj.run()
    except rospy.ROSInterruptException:
        pass
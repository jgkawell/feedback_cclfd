#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32

""" This class classifies the position of the tracked object
    (human's hand or head, etc.). """


class motion():
    def __init__(self):
        # initialize the node
        rospy.init_node('motion', anonymous=True)
        # initialize pub/sub
        self.pub = rospy.Publisher('/classifiers/motion', Bool, queue_size=10)
        rospy.Subscriber("/sensors/mocap", Float32, self.callback)
        self.start = True
        self.pivot = 0
        self.state = True

    def callback(self, data):
        # extract z value from message data
        z = round(data.data[2], 4)

        # setting the pivot
        if self.start:
            self.pivot = z
            self.start = False

        # setting the bounding region and checking if the hand goes out of
        # bounds and publishing it to the synthesizer
        if z < self.pivot - 0.0300:
            rospy.logwarn("OUT OF BOUNDSs")
            self.state = False
        else:
            self.state = True

        # TODO: Return the classification and some confidence
        # value in the classification
        self.pub.publish(self.state)


if __name__ == '__main__':
    try:
        obj = motion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np

from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray

class QueryNLP():

    def __init__(self):
        rospy.init_node('query_nlp')

        # start sub/pub
        rospy.Subscriber("/classifiers/synthesis", Bool, self.query)
        self.constraint_types_pub = rospy.Publisher("/planners/constraint_types", UInt8MultiArray, queue_size=10)
        

    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0

    def query(self, val):

        if val:
            rospy.logwarn("QUERY NLP: Querying user...")

            array = []
            array_msg = UInt8MultiArray(data=array)
            self.constraint_types_pub.publish(array_msg)


if __name__ == '__main__':
    try:
        obj = QueryNLP()
        obj.run()
    except rospy.ROSInterruptException:
        pass
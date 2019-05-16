#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np

from std_msgs.msg import Bool
from rospy.numpy_msg import numpy_msg
from feedback_planners.msg import ConstraintTypes

""" This class queries the user for a plain English explanation about what the robot did wrong. """
class QueryNLP():

    def __init__(self):
        rospy.init_node('query_nlp')

        # start pub/sub
        rospy.Subscriber("/classifiers/synthesis", Bool, self.query)
        self.constraint_types_pub = rospy.Publisher("/planners/constraint_types", numpy_msg(ConstraintTypes), queue_size=10)
        

    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0

    def query(self, val):

        # TODO: Replace this with a speech-to-text listener
        
        # TODO: Add in a component that extracts constraint features from human's response

        if val:
            rospy.loginfo("QUERY NLP: Querying user...")
            msg = 1
            self.constraint_types_pub.publish(msg)


if __name__ == '__main__':
    try:
        obj = QueryNLP()
        obj.run()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np
import abc

from std_msgs.msg import Bool
from rospy.numpy_msg import numpy_msg
from feedback_planners.msg import ConstraintTypes

""" This class queries the user for a plain English explanation about what the robot did wrong.
     It also uses the strategy pattern to switch between the following algorithms:
     1. No query
     2. Targeted Query"""
class QueryNLP():

    def __init__(self, QueryStrategy):
        rospy.init_node('query_nlp')

        self._QueryStrategy = QueryStrategy


        # start pub/sub
        #TODO: Change synthesizer to accomadate the time stamp
        rospy.Subscriber("/classifiers/synthesis", Bool, self.query)

        # self.constraint_types_pub = rospy.Publisher("/planners/constraint_types", numpy_msg(ConstraintTypes), queue_size=10)

    def run(self):
        while(not rospy.is_shutdown()):
            # keep running to check for info on sub
            x = 0







def query(self, val):

        # TODO: Replace this with a text-to-speech api/command
        
        # TODO: Query based on the strategy

        if val:
            rospy.loginfo("QUERY NLP: Querying user...")

            #check the usage

            query_algorithm = NoQuery()
            query_user = QueryNLP(query_algorithm)
            query_user.query_algorithm_interface()




if __name__ == '__main__':
    try:
        obj = QueryNLP()
        obj.run()
    except rospy.ROSInterruptException:
        pass
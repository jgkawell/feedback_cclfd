#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from parsec_cclfd import ParsecCCLfD
from feedback_cclfd.srv import Constraint


class ParsecNode():
    '''
    Wrapper class for PARSEC:
        - adds ros components
        - acts as a service server for constraint_update.py
        - takes user input and finds constraint to add to
            correct a skill
    '''
    def __init__(self):
        rospy.init_node("parsec")
        self.constraint_id = None
        self.parsec = ParsecCCLfD()

    # handle function for add_constraint service
    def handle_add_constraint(self, req):
        # take initial user feedback for skill augmentation
        self.parsec.sentence = str(raw_input("What did I do wrong?\n"))
        # traverse PARSEC tree and find constraint
        self.parsec.tree_nlp()
        id = self.parsec.constraint_id  # take id
        self.parsec.constraint_id = None  # reset id for parsec instance
        return id

    def run(self):
        self.add_constraint_srv = rospy.Service('add_constraint',
                                                Constraint,
                                                self.handle_add_constraint)
        rospy.loginfo("ADD CONSTRAINT: Starting...")
        rospy.spin()


if __name__ == "__main__":
    test = ParsecNode()
    test.parsec.sentence = 'You were too close.'
    test.run()

import rospy
from parsec_cclfd import ParsecCCLfD
from feedback_cclfd.srv import Constraint


class ParsecNode():
    def __init__(self):
        rospy.init_node("parsec")
        # self.constraint_server = AddConstraintServer()
        # self.constraint_server.run()
        self.constraint_id = None
        self.parsec = ParsecCCLfD()

    def handle_add_constraint(self, req):
        # TODO: add functionality to wait until tree is traversed
        self.parsec.sentence = str(raw_input("What did I do wrong?\n"))
        self.parsec.tree_nlp()
        id = self.parsec.constraint_id
        self.parsec.constraint_id = None
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

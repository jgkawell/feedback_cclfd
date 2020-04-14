import rospy
from parsec_cclfd import ParsecCCLfD
from feedback_cclfd.srv import Constraint
from std_msgs.msg import UInt8


class ParsecNode():
    def __init__(self):
        rospy.init_node("parsec")
        # self.constraint_server = AddConstraintServer()
        # self.constraint_server.run()
        self.constraint_id = None
        self.parsec = ParsecCCLfD()

    def handle_add_constraint(self, req):
        # TODO: add functionality to wait until tree is traversed
        print("######################################Request recieved")
        while self.parsec.constraint_id is None:
            continue
        id = UInt8(self.parsec.constraint_id)
        print('#######################################Constraint to update: {}'.format(id.data))
        self.parsec.constraint_id = None
        return id

    def run(self):
        self.parsec.tree_nlp()
        self.add_constraint_srv = rospy.Service('add_constraint',
                                                Constraint,
                                                self.handle_add_constraint)
        rospy.loginfo("ADD CONSTRAINT: Starting...")
        rospy.spin()

    
if __name__ == "__main__":
    test = ParsecNode()
    test.parsec.sentence = 'You should not tip the cup over when it is bringing it over to the person'
    test.run()


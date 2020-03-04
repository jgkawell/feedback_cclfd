import rospy
from feedback_cclfd.srv import Constraint


class AddConstraint:
    def __init__(self):
        rospy.init_node('add_constraint_server')

    def handle_add_constraint(self, req):
        pass

    def run(self):
        s = rospy.Service("add_constraint", Constraint,
                          self.handle_add_constraint)

    def handle_add_two_ints(self, req):
        print "Returning [%s + %s = %s]" % (req.a, req.b, (req.a + req.b))
        return AddTwoIntsResponse(req.a + req.b)

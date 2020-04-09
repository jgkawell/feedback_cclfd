import rospy
from feedback_cclfd.srv import Constraint


class AddConstraintServer:
    def __init__(self):
        rospy.init_node('add_constraint_server')
        self.constraint_id = None

    def handle_add_constraint(self, req):
        # TODO: add functionality to wait until tree is traversed
        while self.constraint_id is None:
            continue
        id = self.constraint_id
        self.constraint_id = None
        return id

    def run(self):
        self.add_constraint_srv = rospy.Service('add_constraint',
                                                Constraint,
                                                self.handle_add_constraint)
        rospy.loginfo("ADD CONSTRAINT: Starting...")
        rospy.spin()


if __name__ == '__main__':
    try:
        obj = AddConstraintServer()
        obj.run()
    except rospy.ROSInterruptException:
        pass

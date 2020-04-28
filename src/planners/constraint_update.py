#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from feedback_cclfd.srv import Constraint
from cairo_lfd_msgs.msg import NodeTime
from std_msgs.msg import UInt8, String
import json


class ConstraintUpdate():
    def __init__(self):
        rospy.init_node("constraint_updater")

        # subscriber to listen to keyframe transitions
        self.node_time_sub = rospy.Subscriber("/lfd/node_time",
                                              NodeTime,
                                              self.node_time_callback)

        # subscriber to listen for user trigger for bad skill
        # assumes trigger is String('True') until false
        self.user_trigger_sub = rospy.Subscriber("user_trigger",
                                                 UInt8,
                                                 self.user_trigger_callback)

        # publisher to publish to constraint update topic in
        # cairo-cclfd/controllers/study_controller.FeedbackLfDStudyController
        self.update_pub = rospy.Publisher('/cairo_lfd/model_update',
                                          String,
                                          queue_size=10)

        # list to hold keyframes that occured durring trigger
        # NOTE: needs to be reset in between updates
        # self.keyframesUpdate = list(range(20)) # NOTE: for testing
        self.keyframesUpdate = []

        # var to hold trigger status
        self.trigger = True

    # callback to listen for keyframes while trigger is true
    def node_time_callback(self, data):
        if self.trigger:
            self.keyframesUpdate.append(int(data.cur_node))
            self.keyframesUpdate.append(int(data.next_node))

    # callback to listen for trigger
    def user_trigger_callback(self, data):
        self.trigger = data.data

    # function to publish to constraint update
    def update_constraints(self):
        # need to wait for all keyframes to be collected and for constraint
        # to update service call to ask for constraint before updating skill

        rospy.wait_for_service('add_constraint')
        try:
            self.add_constraint = rospy.ServiceProxy(
                "add_constraint", Constraint)  # constraint ID to be updated
            resp = self.add_constraint(1)
        except rospy.ServiceException:
            rospy.logwarn("Service setup failed (add_constraint)")
        update_dict = {}

        for keyframe in self.keyframesUpdate:
            update_dict[keyframe] = {"applied_constraints": [resp.constraint]}
        self.update_pub.publish(json.dumps(update_dict))
        # clear stored keyframes and constraint
        self.keyframesUpdate = []


if __name__ == '__main__':
    test = ConstraintUpdate()
    test.update_constraints()

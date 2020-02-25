import rospy

from cairo_lfd_msgs.msg import NodeTime
from std_msgs.msg import String
import json


class ConstraintUpdate():
    def __init__(self):
        rospy.init_node("Constraint Updater")

        # subscriber to listen to keyframe transitions
        self.node_time_sub = rospy.Subscriber("/lfd/node_time",
                                              NodeTime,
                                              self.node_time_callback)

        # subscriber to listen for user trigger for bad skill
        # assumes trigger is String('True') until false
        self.user_trigger_sub = rospy.Subscriber("user_trigger",
                                                 String,
                                                 self.user_trigger_callback)

        # publisher to publish to constraint update topic in
        # cairo-cclfd/controllers/study_controller.FeedbackLfDStudyController
        self.update_pub = rospy.Publisher('/cairo_lfd/model_update',
                                          String,
                                          queue_size=10)

        # list to hold keyframes that occured durring trigger
        self.keyframesUpdate = []  # NOTE: needs to be reset inbetween updates

        # var to hold trigger status
        self.trigger = False

        # constraint to add to keyframes recieved through update_constraint Service
        self.constraint = None


    # callback to add constraint to update
    def add_constraint_callback(self, data):
        self.constraint = data.data

    # callback to listen for keyframes while trigger is true
    def node_time_callback(self, data):
        if self.trigger:
            self.keyframesUpdate.append(int(data.cur_node))
            self.keyframesUpdate.append(int(data.next_node))

    # callback to listen for trigger
    def user_trigger_callback(self, data):
        self.trigger = data.data == 'True'

    # function to publish to constraint update
    def update_constraints(self):
        #need to wait for all keyframes to be collected and for constraint to update
        #service call to ask for constraint before updating skill
        # TODO set up the add_constraint Service
        rospy.wait_for_service('add_constraint')
        try:
            self.constraint = rospy.ServiceProxy(
                "add_constraint", Constraint)#this Constraint object could just be a string, but should just be the constraint ID
        except rospy.ServiceException:
            rospy.logwarn("Service setup failed (add_constraint)")
        update_dict = {}
        for keyframe in self.keyframesUpdate:
            update_dict[keyframe] = {"applied_constraints": [self.constraint]}
        self.update_pub.publish(json.dumps(update_dict))
        #clear stored keyframes and constraint
        self.keyframesUpdate = []
        self.constraint = None

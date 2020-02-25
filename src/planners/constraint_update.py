import rospy

from cairo_lfd_msgs.msg import NodeTime
from std_msgs.msg import String
import json



class ConstraintUpdate():
    def __init__(self):
        rospy.init_node("Constraint Updater")
        #subscriber to listen to keyframe transitions
        self.node_time_sub = rospy.Subscriber("/lfd/node_time",
                         NodeTime,
                         self.node_time_callback)
        #subscriber to listen for user trigger for bad skill
        #assumes trigger is String('True') until false 
        self.user_trigger_sub = rospy.Subscriber("user_trigger",
                        String,
                        self.user_trigger_callback)
        #subscriber to listen to which constraint needs to be updated based on user feedback
        self.constraint_sub = rospy.Subscriber('add_constraint',
                         String,
                         self.add_constraint_callback)
        #publisher to publish to constraint update topic in cairo-cclfd/controllers/study_controller.FeedbackLfDStudyController
        self.update_pub = rospy.Publisher('/cairo_lfd/model_update',
                         String,
                         queue_size=10)
        #list to hold keyframes that occured durring trigger
        self.keyframesUpdate = [] #needs to be reset in between updates
        #var to hold trigger status
        self.trigger = False
        #constraint to add to keyframes recieved through constraint subscriber
        self.constraint = None
    
    #callback to add constraint to update
    def add_constraint_callback(self, data):
        self.constraint = data.data
    #callback to listen for keyframes while trigger is true
    def node_time_callback(self, data):
        if self.trigger:
            self.keyframesUpdate.append(int(data.cur_node))
            self.keyframesUpdate.append(int(data.next_node))
    #callback to listen for trigger
    def user_trigger_callback(self, data):
        self.trigger = data.data == 'True'
    #function to publish to constraint update
    def update(self):
        while self.trigger==True:
            pass
        update_dict = {}
        for keyframe in self.keyframesUpdate:
            update_dict[keyframe] = {"applied_constraints": self.constraint}
        self.update_pub.publish()
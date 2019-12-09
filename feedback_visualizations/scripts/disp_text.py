#!/usr/bin/env python2.7
# license removed for brevity
import rospy
import numpy as np

from time import sleep
from math import pi, sqrt, sin, cos
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose as ROSPose
from rospy_helpers import origin_pose, load_xform_into_pose


def get_text():
    """ Return  """
    # 1. Create model
    txt = Marker()
    # 2. Populate header
    txt.header.stamp = rospy.Time.now()
    txt.header.frame_id = "base"
    # 3. Set marker info for a persistent marker
    txt.ns = "text"
    txt.action = txt.ADD
    txt.type = txt.TEXT_VIEW_FACING
    txt.text = "Speech to Text Display"
    txt.id = 100
    # How long the object should last before being automatically deleted.
    # 0 means forever
    txt.lifetime = rospy.Duration(0)
    # 4. Set marker size
    txt.scale.z = .1
    # 5. Set marker color
    txt.color.a = 1.0
    txt.color.r = 255 / 255
    txt.color.g = 204 / 255
    txt.color.b = 0 / 255
    # 6. Set the text pose
    txt.pose = ROSPose()
    txt.pose.position.x = 0.0
    txt.pose.position.y = 0.0
    txt.pose.position.z = 1.0
    # N. Return text
    return txt


class TextPoser:

    def update_txt(self, msg):
        """ Update the text with the new xform --> pose """
        self.marker.text = msg.data
        self.pub.publish(self.marker)

    def __init__(self, refreshRate=300):
        # 1. Start the node
        rospy.init_node('NODENAME')
        # 2. Set rate
        self.heartBeatHz = refreshRate  # ----------- Node refresh rate [Hz]
        # Best effort to maintain 'heartBeatHz'
        # URL: http://wiki.ros.org/rospy/Overview/Time
        self.idle = rospy.Rate(self.heartBeatHz)
        # 3. Start subscribers and listeners
        rospy.Subscriber("/viz/user_feedback", String, self.update_txt)
        # 4. Start publishers
        self.pub = rospy.Publisher("/viz/markers", Marker, queue_size=100)
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()
        self.marker = get_text()
        self.sent = False

    def run(self):
        # While ROS is running
        while (not rospy.is_shutdown()):

            # 6. Send the persistent marker to RViz
            if not self.sent:
                for i in range(10):
                    self.pub.publish(self.marker)
                    sleep(0.05)
                self.sent = True

            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep()

        # N. Post-shutdown activities
        else:
            rospy.loginfo("Node Shutdown after %d seconds." %
                          rospy.Time.now().to_sec() - self.initTime)


if __name__ == "__main__":
    try:
        obj = TextPoser(300)
        obj.run()
    except rospy.ROSInterruptException:
        pass

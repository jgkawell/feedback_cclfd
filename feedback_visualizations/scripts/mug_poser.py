#!/usr/bin/env python2.7
# license removed for brevity

import rospy
import tf2_ros
import numpy as np

from time import sleep
from math import pi, sqrt, sin, cos
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TransformStamped
from rospy_helpers import origin_pose, load_xform_into_pose


def get_mug():
    """ Return  """
    # 1. Create model
    mug = Marker()
    # 2. Populate header
    mug.header.stamp = rospy.Time.now()
    mug.header.frame_id = "base"
    # 3. Set marker info for a persistent marker
    mug.ns = "mug"
    mug.action = mug.ADD
    mug.type = mug.CYLINDER
    mug.id = 100
    # How long the object should last before being automatically deleted.
    # 0 means forever
    mug.lifetime = rospy.Duration(0)
    # 4. Set marker size
    mug.scale.x = 0.08
    mug.scale.y = 0.08
    mug.scale.z = 0.10
    # 5. Set marker color
    mug.color.a = 1.0
    mug.color.r = 3 / 255.0
    mug.color.g = 252 / 255.0
    mug.color.b = 240 / 255.0
    # 6. Set the mug pose
    mug.pose = origin_pose()
    # N. Return mug
    return mug


class MugPoser:
    """ A_ONE_LINE_DESCRIPTION_OF_THE_NODE """

    def send_pose(self, pPosn, pOrnt, baseFrame, trgtFrame, pTime):
        """ Send a stamped transform with the given data """
        # 1. Create message
        xform = TransformStamped()
        # ~ Transform Header ~
        xform.header.stamp = pTime
        xform.header.frame_id = baseFrame
        xform.child_frame_id = trgtFrame
        # ~ Position ~
        xform.transform.translation.x = pPosn[0]
        xform.transform.translation.y = pPosn[1]
        xform.transform.translation.z = pPosn[2]
        # ~ Orientation ~
        xform.transform.rotation.x = pOrnt[0]
        xform.transform.rotation.y = pOrnt[1]
        xform.transform.rotation.z = pOrnt[2]
        xform.transform.rotation.w = pOrnt[3]
        # 2. Send message
        self.moveOut.sendTransform(xform)

    def update_mug(self, msg):
        """ Update the mug with the new xform --> pose """
        # A. Create a transform from the wrist to the mug
        posn = [0, 0, 0.140]
        ornt = [0, 0.707, 0, 0.707]
        t_i = rospy.Time.now()
        self.send_pose(posn, ornt, "right_hand", "mug_frame", t_i)
        xform = self.tfBuffer.lookup_transform(
            "base", "mug_frame", rospy.Time(0))
        load_xform_into_pose(xform.transform, self.marker.pose)
        self.pub.publish(self.marker)

    def __init__(self, refreshRate=300):
        """ A_ONE_LINE_DESCRIPTION_OF_INIT """
        # 1. Start the node
        rospy.init_node('NODENAME')
        # 2. Set rate
        self.heartBeatHz = refreshRate  # ----------- Node refresh rate [Hz]
        # Best effort to maintain 'heartBeatHz'
        # URL: http://wiki.ros.org/rospy/Overview/Time
        self.idle = rospy.Rate(self.heartBeatHz)
        # 3. Start subscribers and listeners
        rospy.Subscriber("/viz/wristXform", TransformStamped, self.update_mug)
        self.tfBuffer = tf2_ros.Buffer()  # Needed for tf2
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # 4. Start publishers
        self.pub = rospy.Publisher("/viz/markers", Marker, queue_size=100)
        self.moveOut = tf2_ros.TransformBroadcaster()

        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()
        self.marker = get_mug()
        self.sent = False

    def run(self):
        """ A_ONE_LINE_DESCRIPTION_OF_RUNTIME_ACTIVITY """

        # 0. While ROS is running
        while (not rospy.is_shutdown()):

            # self.pub.publish( self.marker )

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
        obj = MugPoser(300)
        obj.run()
    except rospy.ROSInterruptException:
        pass

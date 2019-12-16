#!/usr/bin/env python2.7
# license removed for brevity

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rospy_helpers import unpack_ROS_xform


class FrameListener:

    def __init__(self, refreshRate=300):
        """ Listens to a particular transform and reports it periodically """
        # Start the node
        rospy.init_node('FrameListener')
        # Set rate
        self.heartBeatHz = refreshRate  # ----------- Node refresh rate [Hz]
        # Best effort to maintain 'heartBeatHz'
        # URL: http://wiki.ros.org/rospy/Overview/Time
        self.idle = rospy.Rate(self.heartBeatHz)
        # Start subscribers
        self.tfBuffer = tf2_ros.Buffer()  # Needed for tf2
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Start publishers
        self.pub = rospy.Publisher(
            "/viz/wristXform", TransformStamped, queue_size=10)
        # Init vars
        self.initTime = rospy.Time.now().to_sec()
        self.runCount = 0

    def run(self):
        """ Listen and report transform """

        # While ROS is running
        while (not rospy.is_shutdown()):
            try:
                xform = self.tfBuffer.lookup_transform(
                    "base", "right_hand", rospy.Time(0))
                self.pub.publish(xform)
                self.runCount += 1

                # NOTE: Some time before the proper transform is broadcast
                if 0 and self.runCount % 150 == 0:
                    posn, ornt = unpack_ROS_xform(xform)
                    rospy.loginfo("WRIST FRAME: Received Pose:")
                    rospy.loginfo("WRIST FRAME: Position: {}".format(posn))
                    rospy.loginfo("WRIST FRAME: Orientation: {}".format(ornt))

            except (tf2_ros.TransformException) as err:
                rospy.logwarn("WRIST FRAME: tf2_ros Error! {}".format(err))

            # Wait until the node is supposed to fire next
            self.idle.sleep()

        # Post-shutdown activities
        else:
            rospy.loginfo("Node Shutdown after %d seconds.",
                          rospy.Time.now().to_sec() - self.initTime)


if __name__ == "__main__":
    try:
        refreshRateHz = rospy.get_param('graphics_refresh_rate', 60)
        obj = FrameListener(refreshRateHz)
        obj.run()
    except rospy.ROSInterruptException:
        pass

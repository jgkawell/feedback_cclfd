#!/usr/bin/env python2.7
# license removed for brevity

import rospy
import numpy as np

from time import sleep
from math import pi, sin, cos
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rospy_helpers import origin_pose, unpack_ROS_xform, \
    RollingList, vec_dif_mag, unpack_ROS_pose
from geometry_msgs.msg import TransformStamped

# ~~ Program Constants ~~
_DEFAULTSCALE = 0.01  # Default width of `LINE_STRIP`
_DISTMARGIN = 0.02  # Limit on how close wrist point history can be together
_HISTORYLIMIT = 50  # -- Max number of points of wrist history to display
_PATHNUMLIMIT = 1  # -- Max number of paths to show on the screen
_GREEN = [0/255.0, 255/255.0,   0/255.0]
_BLUE = [66/255.0, 227/255.0, 255/255.0]  # 66, 227, 255
_NAMESPACE = "paths"


class PathDisplay:

    def add_Pose_arr(self, msg):
        """ Add the path to the list of paths """
        nuPath = []
        for pose in msg.poses:
            posn, ornt = unpack_ROS_pose(pose)
            nuPath.append(posn)
        self.add_path(nuPath, _BLUE)

    def update_history(self, msg):
        """ Update the mug with the new xform --> pose """
        # Get the position from the message
        posn, ornt = unpack_ROS_xform(msg)
        dist = vec_dif_mag(posn, self.pathList[0][-1])
        # If the next point is sufficient distance from the last
        if dist >= _DISTMARGIN:
            self.pathList[0].append(posn)

    def __init__(self, refreshRate=300):
        # Start the node
        rospy.init_node('PathDisplay')
        # Set rate
        self.heartBeatHz = refreshRate  # ----------- Node refresh rate [Hz]
        # Best effort to maintain 'heartBeatHz'
        # URL: http://wiki.ros.org/rospy/Overview/Time
        self.idle = rospy.Rate(self.heartBeatHz)

        # Start subscribers and listeners
        rospy.Subscriber("/viz/pointPlans", PoseArray, self.add_Pose_arr)
        rospy.Subscriber("/viz/wristXform", TransformStamped,
                         self.update_history)

        # Start publishers
        self.pubSngl = rospy.Publisher("/viz/markers", Marker, queue_size=100)
        self.pubMany = rospy.Publisher(
            "/viz/mrkr_arr", MarkerArray, queue_size=100)

        # Init vars
        self.initTime = rospy.Time.now().to_sec()
        # NOTE: The first element is ALWAYS the path traced by the wrist
        self.pathList = [RollingList(_HISTORYLIMIT, initVal=[0, 0, 0])]
        self.colrList = [_GREEN]
        self.mrkrArr = MarkerArray()
        self.mrkrDex = 0
        self.hasRun = False

    def get_next_index(self):
        self.mrkrDex += 1
        return self.mrkrDex

    def add_path(self, path, color=_GREEN):
        """ Add a list of points to the list of paths """
        # Add the path and the color
        self.pathList.append(path)
        self.colrList.append(color)
        # If the number of paths have been exceeded
        # then erase the first static path (index 1)
        if (len(self.pathList) - 1) > _PATHNUMLIMIT:
            self.pathList.pop(1)
            self.colrList.pop(1)

    def del_path(self, index):
        """ Remove path from list and the marker array
            NOTE: Static path indices begin at 1 """
        if index > 0:
            self.pathList.pop(index)
            self.colrList.pop(index)

    def create_line_marker(self, ptsList,
                           scale=_DEFAULTSCALE, color=_GREEN,
                           mrkrNS=_NAMESPACE):
        """ Return a marker composed of the points """
        # Create marker
        trace = Marker()
        # Populate header
        trace.header.stamp = rospy.Time.now()
        trace.header.frame_id = "base"
        # Set marker info for a persistent marker
        trace.ns = mrkrNS
        trace.action = trace.ADD
        trace.type = trace.LINE_STRIP
        trace.id = 200 + self.get_next_index()
        # How long the object should last before being automatically deleted.
        # 0 means forever
        trace.lifetime = rospy.Duration(0)
        # Set marker size
        # Line strips: Only scale.x is used and it controls
        # the width of the line segments.
        trace.scale.x = scale
        # Set marker color
        trace.color.a = 1.0
        trace.color.r = color[0]
        trace.color.g = color[1]
        trace.color.b = color[2]
        # Set the marker pose
        trace.pose = origin_pose()
        # Build the points list
        for pnt in ptsList:
            mrkrPnt = Point()
            mrkrPnt.x = pnt[0]
            mrkrPnt.y = pnt[1]
            mrkrPnt.z = pnt[2]
            trace.points.append(mrkrPnt)
        # Return marker
        return trace

    def publish_all(self):
        self.mrkrArr.markers = []
        for pDex, path in enumerate(self.pathList):
            temp = self.create_line_marker(path, color=self.colrList[pDex])
            temp.id = pDex
            self.mrkrArr.markers.append(temp)
        self.pubMany.publish(self.mrkrArr)

    def test_lines(self):
        """ Spam some lines to the screen  """
        rospy.loginfo("PATH DISPLAY: Running the test function")

        def circ_angle(index, N):
            return 2*pi*(index/N)
        N = 100
        radius = 0.75
        ptsList = [[radius*cos(circ_angle(i, N)), radius *
                    sin(circ_angle(i, N)), 0.0] for i in range(N)]
        mrkr = self.create_line_marker(ptsList, mrkrNS="test")
        for i in range(5):
            self.pubSngl.publish(mrkr)
            sleep(0.1)

    def run(self):
        """ Publish all of the currently stored paths """

        # While ROS is running
        while (not rospy.is_shutdown()):
            if 0 and (not self.hasRun):
                self.test_lines()
                self.hasRun = True

            self.publish_all()

            # Wait until the node is supposed to fire next
            self.idle.sleep()

        # Post-shutdown activities
        else:
            rospy.loginfo("Node Shutdown after %d seconds.",
                          rospy.Time.now().to_sec() - self.initTime)


if __name__ == "__main__":
    try:
        refreshRateHz = rospy.get_param('graphics_refresh_rate', 60)
        obj = PathDisplay(refreshRateHz)
        obj.run()
    except rospy.ROSInterruptException:
        pass

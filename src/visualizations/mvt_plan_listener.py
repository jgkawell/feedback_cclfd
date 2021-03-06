#!/usr/bin/env python2.7
# license removed for brevity

import subprocess
import numpy as np
import rospy
import rospkg
import sys

from math import pi, sqrt, sin, cos
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from pyquaternion import Quaternion

# ~ ROS Access Object ~
rospack = rospkg.RosPack()


class MovePlanListener:

    def plan_receiver(self, msg):
        """ Push plan to the visualizer """
        self.planCount += 1
        first = msg.trajectory[0]
        traj = first.joint_trajectory.points
        outMsg = PoseArray()
        for pnt in traj:
            q_i = pnt.positions  # Joint positions
            x_matx = np.array(self.kdl_kin.forward(q_i))
            R = x_matx[0:3, 0:3]
            quat = Quaternion(matrix=R)
            x_i = Pose()
            x_i.position.x = x_matx[0, 3]
            x_i.position.y = x_matx[1, 3]
            x_i.position.z = x_matx[2, 3]
            x_i.orientation.w = quat.elements[0]
            x_i.orientation.x = quat.elements[1]
            x_i.orientation.y = quat.elements[2]
            x_i.orientation.z = quat.elements[3]
            outMsg.poses.append(x_i)
        self.pub.publish(outMsg)

    def __init__(self, refreshRate=300):
        # Start the node
        rospy.init_node('MovPlanListen')
        # Set rate
        self.heartBeatHz = refreshRate  # ----------- Node refresh rate [Hz]
        # Best effort to maintain 'heartBeatHz'
        # URL: http://wiki.ros.org/rospy/Overview/Time

        # Init vars
        self.initTime = rospy.Time.now().to_sec()
        self.kdl_kin = None
        self.planCount = 0

        # Load the robot description
        descPath = rospack.get_path('sawyer_description')
        rospy.loginfo("MOVEIT PLAN: Found the following path for " +
                      "'sawyer_description': %s", descPath)
        command_string = "rosrun xacro xacro --inorder " + \
            str(descPath) + "/urdf/sawyer.urdf.xacro"
        try:
            robot_desc = subprocess.check_output(
                command_string, shell=True, stderr=subprocess.STDOUT)
            rospy.loginfo("MOVEIT PLAN: Xacro load SUCCESS!")

            try:
                robot_urdf = URDF.from_xml_string(robot_desc)
                self.kdl_kin = KDLKinematics(robot_urdf, 'base', 'right_hand')
                rospy.loginfo("MOVEIT PLAN: Kinematic chain SUCCESS! " +
                              "Test Pose:\n", self.kdl_kin.forward(
                                [pi, pi, pi, pi, pi, pi, pi]))
            except Exception as err:
                rospy.logwarn("MOVEIT PLAN: " +
                              "There was a problem reading the URDF: %s",
                              err.message)

        except subprocess.CalledProcessError as process_error:
            rospy.logfatal(
                'Failed to run xacro command: %s' % process_error.output)
            sys.exit(1)

        self.idle = rospy.Rate(self.heartBeatHz)

        # Start subscribers and listeners
        rospy.Subscriber("/move_group/display_planned_path",
                         DisplayTrajectory, self.plan_receiver)

        # Start publishers
        self.pub = rospy.Publisher(
            "/viz/pointPlans", PoseArray, queue_size=100)

    def run(self):
        # While ROS is running
        while (not rospy.is_shutdown()):
            # Wait until the node is supposed to fire next
            self.idle.sleep()

        # Post-shutdown activities
        else:
            rospy.loginfo("Received %d plans in total", self.planCount)
            rospy.loginfo("Node Shutdown after %d seconds.",
                          rospy.Time.now().to_sec() - self.initTime)


if __name__ == "__main__":
    try:
        refreshRateHz = rospy.get_param('graphics_refresh_rate', 60)
        obj = MovePlanListener(refreshRateHz)
        obj.run()
    except rospy.ROSInterruptException:
        pass

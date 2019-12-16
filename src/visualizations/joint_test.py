#!/usr/bin/env python2.7
# license removed for brevity

import rospy
import numpy as np

from time import sleep
from math import pi, sqrt, sin, cos
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rospy_helpers import origin_pose


def circ_angle(index, N):
    return 2*pi*(index/N)


class QTester:
    """ Exercise the joints """

    def __init__(self, refreshRate=300):
        # Start the node
        rospy.init_node('QTester')
        # Set rate
        self.heartBeatHz = refreshRate  # ----------- Node refresh rate [Hz]
        # Best effort to maintain 'heartBeatHz'
        # URL: http://wiki.ros.org/rospy/Overview/Time
        self.idle = rospy.Rate(self.heartBeatHz)
        # Start publishers
        self.pub = rospy.Publisher("/viz/ctrl", JointState, queue_size=10)
        self.pubX = rospy.Publisher(
            "/viz/pointPlans", PoseArray, queue_size=100)
        self.pubQ = rospy.Publisher(
            "/move_group/display_planned_path",
            DisplayTrajectory,
            queue_size=100)
        # Init vars
        self.initTime = rospy.Time.now().to_sec()
        self.qCmd = JointState()
        self.qCmd.header = Header()
        self.qCmd.header.seq = 0
        self.qCmd.header.stamp = rospy.get_rostime()
        self.qCmd.name = ['head_pan', 'right_j0', 'right_j1',
                          'right_j2', 'right_j3', 'right_j4',
                          'right_j5', 'right_j6']
        self.qCmd.position = [0.0 for i in range(len(self.qCmd.name))]
        self.qCmd.header.frame_id = 'pedestal'
        self.Qspeed = pi / 300.0
        # Modes available to command arm
        # int32 POSITION_MODE   = 1
        # int32 VELOCITY_MODE   = 2
        # int32 TORQUE_MODE     = 3
        # int32 TRAJECTORY_MODE = 4
        rospy.loginfo("JOINT TEST: Init completed!")

    def test_planned_paths_X(self):
        """ Spam the system with dummy poses """
        Z = 0.0
        rospy.loginfo("JOINT TEST: Publishing pose plans ...")
        for i in range(3):
            rospy.loginfo("JOINT TEST: %d", i+1)
            Z += 0.100
            arr = PoseArray()
            N = 100
            radius = 0.75
            ptsList = [[radius*cos(circ_angle(i, N)), radius *
                        sin(circ_angle(i, N)), Z] for i in range(N)]
            for pnt in ptsList:
                pose_i = origin_pose()
                pose_i.position.x = pnt[0]
                pose_i.position.y = pnt[1]
                pose_i.position.z = pnt[2]
                arr.poses.append(pose_i)
            self.pubX.publish(arr)
        rospy.loginfo("JOINT TEST: COMPLETE")

    def test_planned_paths_Q(self):
        """ Spam the system with dummy configurations """
        rospy.loginfo("JOINT TEST: Publishing configuration plans")
        starts = [0.0, pi/3.0, 2.5*pi/3.0]
        span = 1.50*pi/6.0
        N = 100
        for i in range(3):
            rospy.loginfo("JOINT TEST: %d", i+1)
            angles = np.linspace(starts[i], starts[i] + span, N)
            traj = DisplayTrajectory()
            rTrj = RobotTrajectory()
            for angle in angles:
                q = [angle for j in range(6)]
                t_i = JointTrajectoryPoint()
                t_i.positions = q
                rTrj.joint_trajectory.points.append(t_i)
            sleep(1.0)
            traj.trajectory.append(rTrj)
            self.pubQ.publish(traj)
        rospy.loginfo("JOINT TEST: COMPLETE")

    def run(self):

        if 0:
            sleep(2.0)
            self.test_planned_paths_X()
        if 1:
            sleep(2.0)
            self.test_planned_paths_Q()

        # While ROS is running
        while (not rospy.is_shutdown()):

            qNu = [self.qCmd.position[i] +
                   self.Qspeed for i in range(len(self.qCmd.name))]
            self.qCmd.position = list(qNu)
            self.qCmd.header.seq += 1
            self.qCmd.header.stamp = rospy.get_rostime()

            self.pub.publish(self.qCmd)

            # Wait until the node is supposed to fire next
            self.idle.sleep()

        # Post-shutdown activities
        else:
            rospy.loginfo("Node Shutdown after %d seconds.",
                          rospy.Time.now().to_sec() - self.initTime)


if __name__ == "__main__":
    try:
        refreshRateHz = rospy.get_param('graphics_refresh_rate', 60)
        obj = QTester(refreshRateHz)
        obj.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
# Future imports must be called before everything else, including
# triple-quote docs!
from __future__ import division

__progname__ = "joint_test.py"
__version__ = "2019.12"
__desc__ = "Exercise the joints"

# ~~~ Imports ~~~
# ~~ Standard ~~
from math import pi, sqrt, sin, cos
# ~~ Special ~~
import os.path
import sys
import rospy
import numpy as np
from intera_core_msgs.msg import JointCommand, EndpointState, \
    EndpointStates, CollisionDetectionState
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

"""
James Watson , Template Version: 2019-03-10
Built on Wing 101 IDE for Python 2.7

Dependencies: numpy , rospy
"""


"""

[ head_pan , right_j0 , right_j1 , right_j2 ,
  right_j3 , right_j4 , right_j5 , right_j6 ]

~~~ Developmnent Plan ~~~
[ ] ITEM1
[ ] ITEM2
"""

# === Init Environment ===================================================
# ~~~ Prepare Paths ~~~
# URL, dir containing source file: http://stackoverflow.com/a/7783326
SOURCEDIR = os.path.dirname(os.path.abspath(__file__))
PARENTDIR = os.path.dirname(SOURCEDIR)
# ~~ Path Utilities ~~


# Might need this to fetch a lib in a parent directory
def prepend_dir_to_path(pathName): sys.path.insert(0, pathName)

# ~~ Local ~~


# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
# http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
infty = 1e309
endl = os.linesep

# ~~ Script Signature ~~


# Return a string representing program name and verions
def __prog_signature__(): return __progname__ + " , Version " + __version__

# ___ End Init ___________________________________________________________


# === Main Application ===================================================

# ~~ Program Constants ~~


# == Program Functions ==


# __ End Func __


# == Program Classes ==

class QTester:
    """ Exercise the joints """

    def __init__(self, refreshRate=300):
        """ A_ONE_LINE_DESCRIPTION_OF_INIT """
        # 1. Start the node
        rospy.init_node('QTester')
        # 2. Set rate
        self.heartBeatHz = refreshRate  # ----------- Node refresh rate [Hz]
        # Best effort to maintain 'heartBeatHz' , URL:
        # http://wiki.ros.org/rospy/Overview/Time
        self.idle = rospy.Rate(self.heartBeatHz)

        # 3. Start subscribers and listeners
        # rospy.Subscriber( "TOPIC_NAME" , MSG_TYPE , CALLBACK_FUNC )

        # 4. Start publishers
        self.pub = rospy.Publisher(
            "/robot/limb/right/joint_command",
            JointCommand,
            queue_size=10)

        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()
        self.qCmd = JointCommand()
        self.qCmd.names = [
            'head_pan',
            'right_j0',
            'right_j1',
            'right_j2',
            'right_j3',
            'right_j4',
            'right_j5',
            'right_j6']
        self.qCmd.position = [0.0 for i in range(len(self.qCmd.names))]
        self.qCmd.mode = 1
        self.qCmd.header = Header()
        self.qCmd.header.seq = 0
        self.qCmd.header.stamp = rospy.get_rostime()
        self.qCmd.header.frame_id = 'base'
        self.Qspeed = 2 * pi / 300.0
        # Modes available to command arm
        # int32 POSITION_MODE   = 1
        # int32 VELOCITY_MODE   = 2
        # int32 TORQUE_MODE     = 3
        # int32 TRAJECTORY_MODE = 4
        print "Init completed!"

    def run(self):
        """ A_ONE_LINE_DESCRIPTION_OF_RUNTIME_ACTIVITY """

        print "About to run! `rospy` running?:", not rospy.is_shutdown()

        # 0. While ROS is running
        while (not rospy.is_shutdown()):

            qNu = [self.qCmd.position[i] +
                   self.Qspeed for i in range(len(self.qCmd.names))]
            self.qCmd.position = list(qNu)
            self.qCmd.header.seq += 1
            self.qCmd.header.stamp = rospy.get_rostime()

            print self.qCmd.position

            if self.qCmd.header.seq % 300 == 0:
                print qNu
                print self.qCmd.position

            self.pub.publish(self.qCmd)

            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep()

        # N. Post-shutdown activities
        else:
            print "\nNode Shutdown after", rospy.Time.now().to_sec() - \
                self.initTime, "seconds"


# __ End Class __


# == Program Vars ==


# __ End Vars __

if __name__ == "__main__":
    print __prog_signature__()
    termArgs = sys.argv[1:]  # Terminal arguments , if they exist

    FOO = QTester(300)
    FOO.run()


# ___ End Main ___________________________________________________________


# === Spare Parts ========================================================


# ___ End Spare __________________________________________________________

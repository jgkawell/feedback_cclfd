#!/usr/bin/env python2.7
# license removed for brevity

import rospy
import numpy as np

from random import random
from geometry_msgs.msg import Point as ROSPntMsg
from geometry_msgs.msg import Pose as ROSPose


def get_ROS_ntime_secs():
    """ The the current time in decimal seconds down to decimal nanosecond """
    now = rospy.get_rostime()
    rtnNum = now.secs + now.nsecs / 1e10
    return rtnNum


def point_msg_2_arr(pntMsg):
    """ Return an array representing 'geometry_msgs/Point' """
    return [pntMsg.x, pntMsg.y, pntMsg.z]


def origin_pose():
    """ Return a no-turn pose at [0,0,0] """
    rtnPose = ROSPose()
    # ~ Position ~
    rtnPose.position.x = 0.0
    rtnPose.position.y = 0.0
    rtnPose.position.z = 0.0
    # ~ Orientation ~
    rtnPose.orientation.w = 1.0
    rtnPose.orientation.x = 0.0
    rtnPose.orientation.y = 0.0
    rtnPose.orientation.z = 0.0
    return rtnPose


def load_pos_into_pose(posArr, poseMsg):
    """ Populate a ROS pose with an R3 list , in place """
    poseMsg.x = posArr[0]
    poseMsg.y = posArr[1]
    poseMsg.z = posArr[2]


def vec_random(dim):
    """ Return a random vector in R-'dim' space with coordinates in [0,1) """
    rtnVec = []
    for i in range(dim):
        rtnVec.append(random())
    return rtnVec


def vec3d_rand_corners(corner1, corner2):
    """ Sample one R3 point from the AABB
        defined by 'corner1' and 'corner2' """
    span = np.subtract(corner2, corner1)
    sample = vec_random(3)
    return [corner1[0]+span[0]*sample[0],
            corner1[1]+span[1]*sample[1],
            corner1[2]+span[2]*sample[2]]


def unpack_ROS_xform(xform):
    """ Unpack the ROS transform message into position and orientation """
    posn = [xform.transform.translation.x,
            xform.transform.translation.y, xform.transform.translation.z]
    ornt = [xform.transform.rotation.x, xform.transform.rotation.y,
            xform.transform.rotation.z, xform.transform.rotation.w]
    return posn, ornt


def unpack_ROS_pose(pose):
    """ Unpack the ROS transform message into position and orientation """
    posn = [pose.position.x, pose.position.y, pose.position.z]
    ornt = [pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w]
    return posn, ornt


def load_xform_into_pose(xform, pose):
    """ Transfer frame data from transform to a pose """
    # Load translation --> position
    pose.position.x = xform.translation.x
    pose.position.y = xform.translation.y
    pose.position.z = xform.translation.z
    # Load rotation --> orientation
    pose.orientation.w = xform.rotation.w
    pose.orientation.x = xform.rotation.x
    pose.orientation.y = xform.rotation.y
    pose.orientation.z = xform.rotation.z


def vec_mag(vec):
    """ Return the magnitude of a vector """
    return np.linalg.norm(vec)


def vec_dif_mag(vec1, vec2):
    """ Return the magnitude of the vector
        difference between 'vec1' and 'vec2' """
    return vec_mag(np.subtract(vec1, vec2))


if __name__ == "__main__":
    for i in xrange(30):
        rospy.loginfo(vec3d_rand_corners([0, 0, 0], [100, 100, 100]))


class RollingList(list):
    """ A rolling window based on 'list' """

    def __init__(self, winLen, initVal=0.0, *args):
        """ Normal 'list' init """
        list.__init__(self, [initVal for i in range(winLen)], *args)

    def append(self, item):
        """ Append an item to the back of the list """
        list.append(self, item)
        del self[0]

    def prepend(self, item):
        """ Prepend an item to the front of the list """
        self.insert(0, item)
        del self[-1]

    def get_average(self):
        """ Get the rolling average
            NOTE: Calling this function after inserting
            non-numeric or non-scalar elements will result in an error """
        return sum(self) * 1.0 / len(self)

    def item_list(self):
        """ Return a copy of the RollingList as a list """
        return self[:]

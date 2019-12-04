#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "rospy_helpers.py"
__version__  = "2019.01" 
__desc__     = "Utilities for writing rospy nodes"
"""
James Watson , Template Version: 2018-05-14
Built on Wing 101 IDE for Python 2.7

Dependencies: rospy , numpy
"""


"""  
~~~ Developmnent Plan ~~~
[ ] ITEM1
[ ] ITEM2
"""

# === Init Environment =====================================================================================================================
# ~~~ Prepare Paths ~~~
import sys, os.path
SOURCEDIR = os.path.dirname( os.path.abspath( __file__ ) ) # URL, dir containing source file: http://stackoverflow.com/a/7783326
PARENTDIR = os.path.dirname( SOURCEDIR )
# ~~ Path Utilities ~~
def prepend_dir_to_path( pathName ): sys.path.insert( 0 , pathName ) # Might need this to fetch a lib in a parent directory

# ~~~ Imports ~~~
# ~~ Standard ~~
from math import pi , sqrt
from random import random
# ~~ Special ~~
import numpy as np
import rospy
from geometry_msgs.msg import Point as ROSPntMsg
from geometry_msgs.msg import Pose  as ROSPose
# ~~ Local ~~

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty   = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep

# ~~ Script Signature ~~
def __prog_signature__(): return __progname__ + " , Version " + __version__ # Return a string representing program name and verions

# ___ End Init _____________________________________________________________________________________________________________________________


# === FUNCTIONS ============================================================================================================================

def get_ROS_ntime_secs():
    """ The the current time in decimal seconds down to decimal nanosecond """
    now = rospy.get_rostime()
    rtnNum = now.secs + now.nsecs / 1e10 
    return rtnNum

def point_msg_2_arr( pntMsg ):
    """ Return an array representing 'geometry_msgs/Point' """
    return [ pntMsg.x , pntMsg.y , pntMsg.z ]

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

def load_pos_into_pose( posArr , poseMsg ):
    """ Populate a ROS pose with an R3 list , in place """
    poseMsg.x = posArr[0]
    poseMsg.y = posArr[1]
    poseMsg.z = posArr[2]

def vec_random( dim ): 
    """ Return a random vector in R-'dim' space with coordinates in [0,1) """
    rtnVec = []
    for i in range(dim):
        rtnVec.append( random() )
    return rtnVec

def vec3d_rand_corners( corner1 , corner2 ):
    """ Sample one R3 point from the AABB defined by 'corner1' and 'corner2' """
    span = np.subtract( corner2 , corner1 )
    sample = vec_random( 3 )
    return [ corner1[0]+span[0]*sample[0] , corner1[1]+span[1]*sample[1] , corner1[2]+span[2]*sample[2] ]

def unpack_ROS_xform( xform ):
    """ Unpack the ROS transform message into position and orientation """
    posn = [ xform.transform.translation.x , xform.transform.translation.y , xform.transform.translation.z ] 
    ornt = [ xform.transform.rotation.x    , xform.transform.rotation.y    , xform.transform.rotation.z    , xform.transform.rotation.w ]
    return posn , ornt

def load_xform_into_pose( xform , pose ):
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
    


# ___ END FUNC _____________________________________________________________________________________________________________________________

if __name__ == "__main__":
    for i in xrange( 30 ):
        print vec3d_rand_corners( [0,0,0] , [100,100,100] )

# === CLASSES ==============================================================================================================================





    

# ___ END CLASS ____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare __________________________________________________
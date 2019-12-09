#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "joint_test.py"
__version__  = "2019.12" 
__desc__     = "Exercise the joints"
"""
James Watson , Template Version: 2019-03-10
Built on Wing 101 IDE for Python 2.7

Dependencies: numpy , rospy
"""


"""  

[ head_pan , right_j0 , right_j1 , right_j2 , right_j3 , right_j4 , right_j5 , right_j6 ]

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
from math import pi , sqrt , sin , cos
from time import sleep
# ~~ Special ~~
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
# ~~ Local ~~
from rospy_helpers import origin_pose

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty   = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep

# ~~ Script Signature ~~
def __prog_signature__(): return __progname__ + " , Version " + __version__ # Return a string representing program name and verions

# ___ End Init _____________________________________________________________________________________________________________________________


# === Main Application =====================================================================================================================

# ~~ Program Constants ~~


# == Program Functions ==

def circ_angle( index , N ):
    return 2*pi*( index/N )

# __ End Func __


# == Program Classes ==

class QTester:
    """ Exercise the joints """
    
    def __init__( self , refreshRate = 300 ):
        """ A_ONE_LINE_DESCRIPTION_OF_INIT """
        # 1. Start the node
        rospy.init_node( 'QTester' ) 
        # 2. Set rate
        self.heartBeatHz = refreshRate # ----------- Node refresh rate [Hz]
        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        
        # 3. Start subscribers and listeners
        # rospy.Subscriber( "TOPIC_NAME" , MSG_TYPE , CALLBACK_FUNC )
        
        # 4. Start publishers
        self.pub  = rospy.Publisher( "/viz/ctrl"                        , JointState         , queue_size =  10 )
        self.pubX = rospy.Publisher( "/viz/pointPlans"                  , PoseArray          , queue_size = 100 )
        self.pubQ = rospy.Publisher( "/move_group/display_planned_path" , DisplayTrajectory  , queue_size = 100 )
        
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()  
        self.qCmd     = JointState()

        self.qCmd.header = Header()
        self.qCmd.header.seq   = 0
        self.qCmd.header.stamp = rospy.get_rostime()

        self.qCmd.name     = [ 'head_pan' , 'right_j0' , 'right_j1' , 'right_j2' , 'right_j3' , 'right_j4' , 'right_j5' , 'right_j6' ]
        self.qCmd.position = [ 0.0 for i in range( len( self.qCmd.name ) ) ]
        
        self.qCmd.header.frame_id = 'pedestal'
        self.Qspeed     = pi / 300.0
        # Modes available to command arm
        # int32 POSITION_MODE   = 1
        # int32 VELOCITY_MODE   = 2
        # int32 TORQUE_MODE     = 3
        # int32 TRAJECTORY_MODE = 4
        print "Init completed!"
        
    def test_planned_paths_X( self ):
        """ Spam the system with dummy poses """
        Z = 0.0
        print "Publishing pose plans ..." ,
        for i in range( 3 ):
            print i+1 ,
            Z += 0.100
            arr = PoseArray()
            N       = 100
            radius  =   0.75
            ptsList = [ [ radius*cos( circ_angle( i , N ) ) , radius*sin( circ_angle( i , N ) ) , Z ] for i in range(N) ]
            for pnt in ptsList:
                pose_i = origin_pose()
                pose_i.position.x = pnt[0]
                pose_i.position.y = pnt[1]
                pose_i.position.z = pnt[2]
                arr.poses.append( pose_i )
            self.pubX.publish( arr )
        print "... COMPLETE"

    def test_planned_paths_Q( self ):
        """ Spam the system with dummy configurations """
        print "Publishing configuration plans ..." ,
        starts = [ 0.0 , pi/3.0 , 2.5*pi/3.0 ]
        span   = 1.50*pi/6.0
        N      = 100
        for i in range( 3 ):
            print i+1 ,
            angles = np.linspace( starts[i] , starts[i] + span , N ) 
            traj   = DisplayTrajectory()
            rTrj   = RobotTrajectory()
            for angle in angles:
                q   = [ angle for j in range(6) ]
                t_i = JointTrajectoryPoint()
                t_i.positions = q
                rTrj.joint_trajectory.points.append( t_i )
            sleep( 1.0 )
            traj.trajectory.append( rTrj )
            self.pubQ.publish( traj )
        print "... COMPLETE"


    def run( self ):
        """ A_ONE_LINE_DESCRIPTION_OF_RUNTIME_ACTIVITY """
        
        print "About to run! `rospy` running?:" , not rospy.is_shutdown()

        if 0:
            sleep( 2.0 )
            self.test_planned_paths_X()
        if 1:
            sleep( 2.0 )
            self.test_planned_paths_Q()

        # 0. While ROS is running
        while ( not rospy.is_shutdown() ):
            
            qNu = [ self.qCmd.position[i] + self.Qspeed for i in range( len( self.qCmd.name ) ) ]
            self.qCmd.position = list( qNu )
            self.qCmd.header.seq += 1
            self.qCmd.header.stamp = rospy.get_rostime()

            print self.qCmd.position

            if self.qCmd.header.seq % 300 == 0:
                print qNu
                print self.qCmd.position

            self.pub.publish( self.qCmd )
            
            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep()        
        
        # N. Post-shutdown activities
        else:
            print "\nNode Shutdown after" , rospy.Time.now().to_sec() - self.initTime , "seconds"
        

# __ End Class __


# == Program Vars ==



# __ End Vars __


if __name__ == "__main__":
    print __prog_signature__()
    termArgs = sys.argv[1:] # Terminal arguments , if they exist
    
    refreshRateHz = rospy.get_param( 'graphics_refresh_rate' , 60 )
    FOO = QTester( refreshRateHz )
    FOO.run()    
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================
"""

from intera_core_msgs.msg import (
    JointCommand,
    EndpointState,
    EndpointStates,
    CollisionDetectionState,
)

"""
# ___ End Spare ____________________________________________________________________________________________________________________________
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "mvt_plan_listener.py"
__version__  = "2019.12" 
__desc__     = "A_ONE_LINE_DESCRIPTION_OF_THE_FILE"
"""
James Watson , Template Version: 2019-03-10
Built on Wing 101 IDE for Python 2.7

Dependencies: numpy , rospy
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
from math import pi , sqrt , sin , cos
import subprocess
# ~~ Special ~~
import numpy as np
import rospy , rospkg
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
# ~ ROS Access Object ~
rospack = rospkg.RosPack()

# ~~ Local ~~

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty   = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep

# ~~ Script Signature ~~
def __prog_signature__(): return __progname__ + " , Version " + __version__ # Return a string representing program name and verions

# ___ End Init _____________________________________________________________________________________________________________________________


# === Main Application =====================================================================================================================

# ~~ Program Constants ~~


# == Program Classes ==

class MovePlanListener:
    """ A_ONE_LINE_DESCRIPTION_OF_THE_NODE """
    
    def plan_receiver( self, msg ):
        """ Push plan to the visualizer """
        traj = msg.trajectory.joint_trajectory.points
        outMsg = PoseArray()
        for pnt in traj:
            q_i = pnt.positions
            x_i = Pose()
            x_i.position.x = q_i[0][3]
            x_i.position.y = q_i[1][3]
            x_i.position.z = q_i[2][3]
            x_i.orientation.w = 1.0
            x_i.orientation.x = 0.0
            x_i.orientation.y = 0.0
            x_i.orientation.z = 0.0
            outMsg.poses.append( x_i )
        self.pub.publish( outMsg )

    def __init__( self , refreshRate = 300 ):
        """ A_ONE_LINE_DESCRIPTION_OF_INIT """
        # 1. Start the node
        rospy.init_node( 'MovPlanListen' ) 
        # 2. Set rate
        self.heartBeatHz = refreshRate # ----------- Node refresh rate [Hz]
        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        
        # 3. Start subscribers and listeners
        rospy.Subscriber( "/move_group/display_planned_path" , DisplayTrajectory , self.plan_receiver )
        
        # 4. Start publishers
        self.pub = rospy.Publisher( "/viz/pointPlans" , PoseArray , queue_size = 100 )
        
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()  
        self.kdl_kin  = None

        # 6. Load the robot description
        descPath = rospack.get_path('sawyer_description')
        print "Found the following path for 'sawyer_description':" , descPath
        command_string = "rosrun xacro xacro --inorder " + str( descPath ) + "/urdf/sawyer.urdf.xacro"
        try:
            robot_desc = subprocess.check_output( command_string , shell=True , stderr=subprocess.STDOUT )
            print "Xacro load SUCCESS!"

            try:
                robot_urdf = URDF.from_xml_string( robot_desc )
                self.kdl_kin = KDLKinematics( robot_urdf , 'base' , 'right_wrist' )
                print "Kinematic chain SUCCESS! , Test Pose:\n" , self.kdl_kin.forward( [ pi , pi , pi , pi , pi , pi ] )
            except Exception as err:
                print "There was a problem reading the URDF: \n\t" , err

        except subprocess.CalledProcessError as process_error:
            rospy.logfatal( 'Failed to run xacro command with error: \n%s' , process_error.output )
            sys.exit(1)
        
    def run( self ):
        """ A_ONE_LINE_DESCRIPTION_OF_RUNTIME_ACTIVITY """
        
        # 0. While ROS is running
        while ( not rospy.is_shutdown() ):
            
            # 1. FIXME: THINGS TO DO WHILE THE NODE IS RUNNING
            
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
    
    FOO = MovePlanListener( 300 )
    FOO.run()    
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare ____________________________________________________________________________________________________________________________
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "path_display.py"
__version__  = "2019.12" 
__desc__     = "Show paths that the robot might take"
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
# ~~ Special ~~
import numpy as np
import rospy , rospkg
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
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

class PathDisplay:
    """ A_ONE_LINE_DESCRIPTION_OF_THE_NODE """
    
    def add_path_arr( self , msg ):
        """ Add the path to the list of paths """

    def __init__( self , refreshRate = 300 ):
        """ A_ONE_LINE_DESCRIPTION_OF_INIT """
        # 1. Start the node
        rospy.init_node( 'PathDisplay' ) 
        # 2. Set rate
        self.heartBeatHz = refreshRate # ----------- Node refresh rate [Hz]
        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        
        # 3. Start subscribers and listeners
        rospy.Subscriber( "/viz/pointPlans" , PoseArray , CALLBACK_FUNC )
        
        # 4. Start publishers
        # self.pub = rospy.Publisher( "TOPIC_NAME" , MSG_TYPE , queue_size = 10 )
        
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()
        self.pathList = []
        self.mrkrArr  = MarkerArray()

    def add_path( self , path , color = [ 255/255.0 , 0/255.0 , 0/255.0 ] ):
        """ Add a list of points to the list of paths """
        # FIXME : START HERE

    def del_path( self , index ):
        """ Remove path from list and the marker array """
        pass
        
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
    
    FOO = PathDisplay( 300 )
    FOO.run()    
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare ____________________________________________________________________________________________________________________________
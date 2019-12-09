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
from time import sleep
# ~~ Special ~~
import numpy as np
import rospy , rospkg
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rospy_helpers import origin_pose , unpack_ROS_xform , RollingList , vec_dif_mag , unpack_ROS_pose
from geometry_msgs.msg import TransformStamped
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
_DEFAULTSCALE =   0.01 # Default width of `LINE_STRIP`
_DISTMARGIN   =   0.02 # Limit on how close wrist point history can be together
_HISTORYLIMIT =  25 # -- Max number of points of wrist history to display
_PATHNUMLIMIT =  10 # -- Max number of paths to show on the screen
_GREEN        = [ 0/255.0 , 255/255.0 , 0/255.0 ]
_NAMESPACE    = "paths"

# == Program Classes ==

class PathDisplay:
    """ A_ONE_LINE_DESCRIPTION_OF_THE_NODE """
    
    def add_Pose_arr( self , msg ):
        """ Add the path to the list of paths """
        nuPath = []
        for pose in msg.poses:
            posn , ornt = unpack_ROS_pose( pose )
            nuPath.append( posn )
        self.add_path( nuPath )
    
    def update_history( self , msg ):
        """ Update the mug with the new xform --> pose """
        # 1. Get the position from the message
        posn , ornt = unpack_ROS_xform( msg )
        dist = vec_dif_mag( posn , self.pathList[0][-1] )
        # print "Got" , posn
        # 2. If the next point is sufficient distance from the last
        if dist >= _DISTMARGIN:
            self.pathList[0].append( posn )
            # print "Append , There are" , len( self.pathList[0] ) , "elements"

    def __init__( self , refreshRate = 300 ):
        """ A_ONE_LINE_DESCRIPTION_OF_INIT """
        # 1. Start the node
        rospy.init_node( 'PathDisplay' ) 
        # 2. Set rate
        self.heartBeatHz = refreshRate # ----------- Node refresh rate [Hz]
        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        
        # 3. Start subscribers and listeners
        rospy.Subscriber( "/viz/pointPlans" , PoseArray        , self.add_Pose_arr   )
        rospy.Subscriber( "/viz/wristXform" , TransformStamped , self.update_history )
        
        # 4. Start publishers
        self.pubSngl = rospy.Publisher( "/viz/markers"  , Marker      , queue_size = 100 )
        self.pubMany = rospy.Publisher( "/viz/mrkr_arr" , MarkerArray , queue_size = 100 )
        
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()
        self.pathList = [ RollingList( _HISTORYLIMIT , initVal = [0,0,0] ) ] # NOTE: The first element is ALWAYS the path traced by the wrist
        self.colrList = [ _GREEN ]
        self.mrkrArr  = MarkerArray()
        self.mrkrDex  = 0
        self.hasRun   = False

    def get_next_index( self ): self.mrkrDex += 1; return self.mrkrDex

    def add_path( self , path , color = _GREEN ):
        """ Add a list of points to the list of paths """
        # 1. Add the path and the color
        self.pathList.append( path  )
        self.colrList.append( color )
        # 2. If the number of paths have been exceeded, then erase the first static path (index 1)
        if ( len( self.pathList ) - 1 ) > _PATHNUMLIMIT:
            self.pathList.pop(1)
            self.colrList.pop(1)

    def del_path( self , index ):
        """ Remove path from list and the marker array , NOTE: Static path indices begin at 1 """
        if index > 0:
            self.pathList.pop( index )
            self.colrList.pop( index )

    def create_line_marker( self , ptsList , scale = _DEFAULTSCALE , color = _GREEN , mrkrNS = _NAMESPACE ):
        """ Return a marker composed of the points """
        # 1. Create marker
        trace = Marker()
        # 2. Populate header
        trace.header.stamp    = rospy.Time.now()
        trace.header.frame_id = "base"
        # 3. Set marker info for a persistent marker
        trace.ns       = mrkrNS
        trace.action   = trace.ADD
        trace.type     = trace.LINE_STRIP
        trace.id       = 200 + self.get_next_index()
        trace.lifetime = rospy.Duration(0) # How long the object should last before being automatically deleted.  0 means forever
        # 4. Set marker size
        trace.scale.x = scale # Line strips: Only scale.x is used and it controls the width of the line segments. 
        # 5. Set marker color
        trace.color.a = 1.0
        trace.color.r = color[0]
        trace.color.g = color[1]
        trace.color.b = color[2]
        # 6. Set the marker pose
        trace.pose = origin_pose()
        # 7. Build the points list
        for pnt in ptsList:
            mrkrPnt = Point()
            mrkrPnt.x = pnt[0]
            mrkrPnt.y = pnt[1]
            mrkrPnt.z = pnt[2]
            trace.points.append( mrkrPnt )
        # N. Return marker
        return trace

    def publish_all( self ):
        self.mrkrArr.markers = []
        for pDex , path in enumerate( self.pathList ):
            temp = self.create_line_marker( path , color = self.colrList[ pDex ] )
            temp.id = pDex
            self.mrkrArr.markers.append( temp )
        self.pubMany.publish( self.mrkrArr )
        
    def test_lines( self ):
        """ Spam some lines to the screen  """
        print "Running the test function ..."
        def circ_angle( index , N ):
            return 2*pi*( index/N )
        N       = 100
        radius  =   0.75
        ptsList = [ [ radius*cos( circ_angle( i , N ) ) , radius*sin( circ_angle( i , N ) ) , 0.0 ] for i in range(N) ]
        mrkr    = self.create_line_marker( ptsList , mrkrNS = "test" )
        for i in range( 5 ):
            self.pubSngl.publish( mrkr )
            sleep( 0.1 )

    def run( self ):
        """ Publish all of the currently stored paths """
        
        # 0. While ROS is running
        while ( not rospy.is_shutdown() ):
            
            if 0 and ( not self.hasRun ):
                self.test_lines()
                self.hasRun = True

            self.publish_all()
            
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
    FOO = PathDisplay( refreshRateHz )
    FOO.run()    
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare ____________________________________________________________________________________________________________________________
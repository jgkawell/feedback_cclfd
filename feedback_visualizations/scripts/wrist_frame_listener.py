#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "wrist_frame_listener.py"
__version__  = "2019.12" 
__desc__     = "Listens to a dynamic tf frame"
"""
James Watson , Template Version: 2019-02-21
Built on Wing 101 IDE for Python 2.7

Dependencies: numpy
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
import math
from math import pi , sqrt
# ~~ Special ~~
import numpy as np
import rospy 
import tf2_ros
from geometry_msgs.msg import TransformStamped
# ~~ Local ~~
from rospy_helpers import unpack_ROS_xform

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



# __ End Func __


# == Program Classes ==

class FrameListener:
    
    def __init__( self , refreshRate = 300 ):
        """ Listens to a particular transform and reports it periodically """
        # 1. Start the node
        rospy.init_node( 'FrameListener' ) 
        # 2. Set rate
        self.heartBeatHz = refreshRate # ----------- Node refresh rate [Hz]
        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        # 3. Start subscribers
        self.tfBuffer = tf2_ros.Buffer() # Needed for tf2
        self.listener = tf2_ros.TransformListener( self.tfBuffer )
        # 4. Start publishers
        self.pub     = rospy.Publisher( "/viz/wristXform" , TransformStamped , queue_size = 10 )
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec()   
        self.runCount = 0 
        
    def run( self ):
        """ Listen and report transform """
        
        # 1. While ROS is running
        while  ( not rospy.is_shutdown() ):
            
            # 1.
            try:
                xform = self.tfBuffer.lookup_transform( "base" , "right_hand" , rospy.Time(0) )
                self.pub.publish( xform )
                self.runCount += 1

                # NOTE: There a a few seconds before the proper transform is broadcast
                if 0 and self.runCount % 150 == 0:
                    posn , ornt = unpack_ROS_xform( xform )
                    print "Received Pose , Position:" , posn , ", Orientation:" , ornt

                
            except ( tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException ) as err:
                print "WARN: tf2_ros Error!" , err
            
            # N-1: Wait until the node is supposed to fire next
            self.idle.sleep() 
        
        # N. Post-shutdown activities
        else:
            print "Node Shutdown after" , rospy.Time.now().to_sec() - self.initTime , "seconds"

# __ End Class __


# == Program Vars ==



# __ End Vars __


if __name__ == "__main__":
    print __prog_signature__()
    termArgs = sys.argv[1:] # Terminal arguments , if they exist
    
    refreshRateHz = rospy.get_param( 'graphics_refresh_rate' , 60 )
    FOO = FrameListener( refreshRateHz )
    FOO.run()    
    
           
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================
"""

posn , ornt = unpack_ROS_xform( xform )
print "Received Pose , Position:" , posn , ", Orientation:" , ornt
t_i = rospy.Time.now()
print >> self.outfile , t_i.to_sec()                
self.runCount += 1

            self.outfile.close()
            _LATENCY = 0.003
            
            # 1. Read the running interval of the broadcasting node
            numPts = 100
            rawTimes = []
            #with open( os.path.join( os.getcwd() , '..' , '..' , '..' , 'time_interval.txt' ) ) as f:
            with open( os.path.join( os.getcwd() , 'time_interval.txt' ) ) as f:
                for line in f:
                    try:
                        t_i = float( line.strip() ) ; print "time: " , t_i 
                        rawTimes.append( t_i )
                    except ValueError:
                        #print "Could not parse" , line , "into a float"
                        pass # Please, don't make so much noise
                # Closes automatically    
            
            # 3. Get a list of times in the past to query
            queryTimes = [ rospy.Time.from_sec( t + _LATENCY ) for t in rawTimes ]
            if 0:
                for t_i in queryTimes:
                    print t_i
            
            # 4. Get the poses at those times
            
            #tfBuffer = tf2_ros.Buffer() # Needed for tf2
            #listener = tf2_ros.TransformListener( tfBuffer )
            
            elemSkip = 10
            
            for i , t_i in enumerate( queryTimes ):
                try:
                    if not ( i % elemSkip ):
                        xform = self.tfBuffer.lookup_transform( "base" , "right_wrist" , t_i )
                        posn , ornt = unpack_ROS_xform( xform )            
                        print "Replay from time" , t_i , "\n\tPosition:" , posn , ", Orientation:" , ornt , endl  
                except ( tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException ) as ex:
                    #print "WARN: tf2 Error:" , ex
                    pass # Please, don't make so much noise

"""
# ___ End Spare ____________________________________________________________________________________________________________________________
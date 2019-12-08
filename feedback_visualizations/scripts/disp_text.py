#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ~~ Future First ~~
from __future__ import division # Future imports must be called before everything else, including triple-quote docs!

__progname__ = "disp_text.py"
__version__  = "2019.12" 
__desc__     = "Display text from Speech to Text"
"""
Christine Chang version of
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
import rospy 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
# ~~ Local ~~
from rospy_helpers import origin_pose , load_xform_into_pose

# ~~ Constants , Shortcuts , Aliases ~~
EPSILON = 1e-7
infty   = 1e309 # URL: http://stackoverflow.com/questions/1628026/python-infinity-any-caveats#comment31860436_1628026
endl    = os.linesep

# ~~ Script Signature ~~
def __prog_signature__(): return __progname__ + " , Version " + __version__ # Return a string representing program name and verions

# ___ End Init _____________________________________________________________________________________________________________________________


# === Main Application =====================================================================================================================

# ~~ Program Constants ~~


# == Program Assets ==

def get_text():
    """ Return  """
    # 1. Create model
    txt = Marker()
    # 2. Populate header
    txt.header.stamp    = rospy.Time.now()
    txt.header.frame_id = "base"
    # 3. Set marker info for a persistent marker
    txt.ns       = "text"
    txt.action   = txt.ADD
    txt.type     = txt.TEXT_VIEW_FACING
    txt.id       = 100
    txt.lifetime = rospy.Duration(0) # How long the object should last before being automatically deleted.  0 means forever
    # 4. Set marker size
    txt.scale.z = 0.5
    # 5. Set marker color
    txt.color.a = 1.0
    txt.color.r = 255 /255
    txt.color.g = 204 /255
    txt.color.b = 0 /255
    # 6. Set the text pose
    txt.pose = origin_pose() #  TODO: Determine appropriate location - may not exist for this Marker type?
    # N. Return text
    return txt

# __ End Assets __


# == Program Classes ==

class TextPoser:
    """ A_ONE_LINE_DESCRIPTION_OF_THE_NODE """

    def update_txt(self, msg):
        """ Update the text with the new xform --> pose """
        load_xform_into_pose( txt.transform , self.marker.pose )
        self.pub.publish(self.marker)
    
    def __init__( self , refreshRate = 300 ):
        """ A_ONE_LINE_DESCRIPTION_OF_INIT """
        # 1. Start the node
        rospy.init_node( 'NODENAME' ) 
        # 2. Set rate
        self.heartBeatHz = refreshRate # ----------- Node refresh rate [Hz]
        self.idle = rospy.Rate( self.heartBeatHz ) # Best effort to maintain 'heartBeatHz' , URL: http://wiki.ros.org/rospy/Overview/Time        
        
        # 3. Start subscribers and listeners
        rospy.Subscriber("/viz/user_feedback", string, self.update_txt)
        
        # 4. Start publishers
        self.pub = rospy.Publisher( "/viz/markers" , Marker , queue_size = 100 )
        
        # 5. Init vars
        self.initTime = rospy.Time.now().to_sec() 
        self.marker   = get_mug()
        self.sent     = False

    def run( self ):
        """ A_ONE_LINE_DESCRIPTION_OF_RUNTIME_ACTIVITY """
        
        # 0. While ROS is running
        while ( not rospy.is_shutdown() ):
            
            # self.pub.publish( self.marker )

            # 6. Send the persistent marker to RViz
            if not self.sent:
                for i in range(10):
                    self.pub.publish( self.marker )
                    sleep( 0.05 )
                self.sent = True

            
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
    
    FOO = TextPoser( 300 )
    FOO.run()    
    

# ___ End Main _____________________________________________________________________________________________________________________________


# === Spare Parts ==========================================================================================================================



# ___ End Spare ____________________________________________________________________________________________________________________________
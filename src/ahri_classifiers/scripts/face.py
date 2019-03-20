#!/usr/bin/env python
# license removed for brevity
import rospy

def face():
    rospy.logwarn("FACE: Not implemented!")
    
if __name__ == '__main__':
    try:
        face()
    except rospy.ROSInterruptException:
        pass
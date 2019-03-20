#!/usr/bin/env python
# license removed for brevity
import rospy

def motion():
    rospy.logwarn("MOTION: Not implemented!")
    
if __name__ == '__main__':
    try:
        motion()
    except rospy.ROSInterruptException:
        pass
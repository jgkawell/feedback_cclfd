#!/usr/bin/env python
# license removed for brevity
import rospy

def demonstrator():
    rospy.logwarn("DEMONSTRATOR: Not implemented!")
    
if __name__ == '__main__':
    try:
        demonstrator()
    except rospy.ROSInterruptException:
        pass
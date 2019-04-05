#!/usr/bin/env python
# license removed for brevity
import rospy

def mocap():
    rospy.logwarn("MOCAP: Not implemented!")
    
if __name__ == '__main__':
    try:
        mocap()
    except rospy.ROSInterruptException:
        pass
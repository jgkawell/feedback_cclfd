#!/usr/bin/env python
# license removed for brevity
import rospy

def camera():
    rospy.logwarn("CAMERA: Not implemented!")
    
if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
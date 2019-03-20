#!/usr/bin/env python
# license removed for brevity
import rospy

def synthesizer():
    rospy.logwarn("SYNTHESIZER: Not implemented!")
    
if __name__ == '__main__':
    try:
        synthesizer()
    except rospy.ROSInterruptException:
        pass
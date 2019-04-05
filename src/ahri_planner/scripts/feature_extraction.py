#!/usr/bin/env python
# license removed for brevity
import rospy

def feature_extraction():
    rospy.logwarn("FEATURE EXTRACTION: Not implemented!")
    
if __name__ == '__main__':
    try:
        feature_extraction()
    except rospy.ROSInterruptException:
        pass
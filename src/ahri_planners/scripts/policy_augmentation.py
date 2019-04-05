#!/usr/bin/env python
# license removed for brevity
import rospy

def policy_augmentation():
    rospy.logwarn("POLICY AUGMENTATION: Not implemented!")
    
if __name__ == '__main__':
    try:
        policy_augmentation()
    except rospy.ROSInterruptException:
        pass
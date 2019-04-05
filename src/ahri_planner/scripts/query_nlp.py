#!/usr/bin/env python
# license removed for brevity
import rospy

def query_nlp():
    rospy.logwarn("QUERY NLP: Not implemented!")
    
if __name__ == '__main__':
    try:
        query_nlp()
    except rospy.ROSInterruptException:
        pass
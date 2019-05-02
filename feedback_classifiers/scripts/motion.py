#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32

class motion():
    def __init__(self):
        rospy.init_node('motion', anonymous=True)
        self.pub = rospy.Publisher('/classifiers/motion', Bool, queue_size=10)
        rospy.Subscriber("/sensors/mocap", Float32, self.callback)
        self.start = True
        self.pivot = 0
        self.state = True

    def callback(self, data):
        z=data.data
        if(self.start==True):
            self.pivot = z 
            self.start=False
            
        if(z<self.pivot-0.0300):
            self.state = False
        else:
            self.state = True
        
        self.pub.publish(self.state)
    
if __name__ == '__main__':
    try:
        obj = motion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
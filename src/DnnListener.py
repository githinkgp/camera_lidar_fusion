# DNN detection Listener Class
import rospy
import numpy as np
from dnn_detect.msg import DetectedObjectArray

# begin class def
class DnnListener:

    def __init__(self):

        #self.tStamp = 0.0
        self.seq = 0
        self.PersonCount = 0
        self.DetectedPerson = []
        
        print 'initializing DNN detection subscriber...'
        self.subDnn = rospy.Subscriber('/dnn_objects',DetectedObjectArray,self.callback)

    # end of init

    def callback(self,msg):

        self.DetectedPerson = []
        #self.tStamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**(-9)
        self.seq = msg.header.seq
        self.PersonCount = 0

        # find the person objects
        for DetectedObj in msg.objects:
         
            ObjectClass = DetectedObj.class_name
            positions = (DetectedObj.x_min, DetectedObj.x_max, DetectedObj.y_min, DetectedObj.y_max)

            if ObjectClass == 'person' and DetectedObj.confidence > 0.6:
                self.DetectedPerson.append(positions)
                self.PersonCount += 1

    # end of callback

# end of class

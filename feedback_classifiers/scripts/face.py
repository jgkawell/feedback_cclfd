#!/usr/bin/env python2.7
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from keras.models import Sequential
from keras.layers import Dense
from keras.models import model_from_json
import os
import numpy as np

class face():
  
    def __init__(self):
        rospy.init_node('face', anonymous=True)
        #loading the model
        self.json_file = open('fer.json', 'r')
        self.loaded_model_json = self.json_file.read()
        self.json_file.close()
        self.loaded_model = model_from_json(self.loaded_model_json)
        self.loaded_model._make_predict_function()
        self.counter =0
        self.emot ={0:0,1:0}
        # load weights into new model
        self. loaded_model.load_weights("fer.h5")

        print("Loaded model from disk")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/sensors/camera", Image, self.callback)
        self.image_pub = rospy.Publisher('/classifiers/face', String, queue_size=10)
        
   

    def callback(self, data):
        labels = ['Neutral','Negative']
        prev_emotion = 0
        self.counter+=1
        print(self.counter)
        
        
        try:
            full_size_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            rospy.logwarn("FACE: Image Recieved...")
        except CvBridgeError as e:
            print(e)
            
        rospy.logwarn("FACE: Classifying emotion...")
        print("Classifier: Image Loaded")
        gray = cv2.cvtColor(full_size_image, cv2.COLOR_BGR2GRAY)
        face = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        faces = face.detectMultiScale(gray, 1.3  , 10)
        for (x, y, w, h) in faces:
            roi_gray = gray[y:y + h, x:x + w]
            cropped_img = np.expand_dims(np.expand_dims(cv2.resize(roi_gray, (48, 48)), -1), 0)
            cv2.normalize(cropped_img, cropped_img, alpha=0, beta=1, norm_type=cv2.NORM_L2, dtype=cv2.CV_32F)
            cv2.rectangle(full_size_image, (x, y), (x + w, y + h), (0, 255, 0), 1)
            if(self.counter%1==0):
                #predicting the emotion
                yhat= self.loaded_model.predict(cropped_img)
                print('yhat:',yhat[0])
                index = np.argmax(yhat[0])
                print('index',index)
                print("Emotion: ",labels[index]) 
                self.emot[index]+=1
                if(self.counter%10==0):
                    emotion = max(self.emot, key=self.emot.get)
                    prev_emotion = emotion
                    self.emot={0:0,1:0}
                    print("index 15: ",emotion)
                    cv2.putText(full_size_image, labels[emotion], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
                    self.image_pub.publish(labels[emotion])
                    rospy.logwarn("FACE: I published %s",labels[emotion])
                else:
                    cv2.putText(full_size_image, labels[prev_emotion], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow('output-face',full_size_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
    
if __name__ == '__main__':
    try:
        obj = face()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
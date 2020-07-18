#!/usr/bin/env python 
# license removed for brevity

import rospy 
from cyberpod_sim_ros.msg import * 
import numpy as np
import scipy as sp 
import sklearn.metrics
from sklearn.metrics.pairwise import euclidean_distances, rbf_kernel
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time

class regression_perception: 

    state_image_ = state()
    state_predicted_ = state()

    def __init__(self):
        # Init pubs, subs
        self.sub_state_image_     = rospy.Subscriber('/camera1/camera_feed', Image, self.imageCallback )
        self.sub_state_true_      = rospy.Subscriber('/cyberpod_sim_ros/state_true', state, self.otherSensorsCallback)
        self.pub_state_predicted_ = rospy.Publisher('state_measured', state, queue_size=1) 
        self.bridge = CvBridge()
        self.load_model()

    
    def load_model(self):
        train_data_fn = 'gridded_data.csv'
        test_data = 'uniform'

        # loading model parameters
        data = np.load('/home/rkcosner/Documents/Caltech/Research/catkin_ws/src/cyberpod_sim_ros/data/coeff_train{}_test{}.npz'.format(train_data_fn, test_data))
        self.coeff = data['coeff'] # n_train by n_target
        self.gamma = data['gamma'] # scalar parameter for RBF kernel
        self.Xs_train = data['Xs_train'] # n_train by n_features training data

    def predict(self, Xs):
        # compute exp(-gamma * ||Xs - Xs_train||^2)
        # K = euclidean_distances(Xs, self.Xs_train, squared=True) # fast implementation
        # K *= -self.gamma
        # np.exp(K, K)  # exponentiate K in-place
        t = time.time()
        K = rbf_kernel(Xs, self.Xs_train, gamma=self.gamma)
        
        rospy.loginfo("Evaluate Kernel: %4f ", time.time() - t )

        return np.dot(K, self.coeff)
    
    def imageCallback(self, data):
        # Update with Predictions
        self.state_image_ = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.state_image_.reshape(1,-1)

        prediction = self.predict(self.state_image_.reshape(1, -1))[0]
        self.state_predicted_.stateVec[0] = prediction[0]
        self.state_predicted_.stateVec[5] = prediction[1]
        
        self.pub_state_predicted_.publish(self.state_predicted_)
        
        rospy.loginfo(self.state_predicted_.stateVec)
   
    def otherSensorsCallback(self, data): 
        # Receive Other Measurements
        self.state_predicted_ = data
        self.state_predicted_.stateVec = list(data.stateVec)
def run_perception():
    perception = regression_perception()


    rospy.init_node('perception', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown(): 
        rate.sleep()

if __name__ == "__main__": 
    try: 
        run_perception()
    except rospy.ROSInterruptException: 
        pass
#!/usr/bin/env python 
# license removed for brevity

import rospy 
from cyberpod_sim_ros.msg import * 
import numpy as np
import scipy as sp 

class measurement_safety_filter: 
    
    inputDes_ = input()
    inputAct_ = input()
    state_image_ = state()
    state_measured_ = state()
    filter_info_ = filterInfo()

    def __init__(self):
        # Set input desired with dummy initial value
        self.inputDes_.inputVec = [0,0]

        # Init pubs, subs
        self.sub_state_image_ = rospy.Subscriber('state_image', state, self.imageCallback )
        self.sub_state_true_  = rospy.Subscriber('state_image', state, self.stateCallback )
        self.sub_inputDes_    = rospy.Subscriber('inputDes', input, self.inputCallback )

        self.pub_inputAct_ = rospy.Publisher('input', input, queue_size=1) 
        self.pub_info_     = rospy.Publisher('safety_filter_info', filterInfo, queue_size=1)

    # Define Callback Functions 
    def filterInput(self, prefilter_input): 
        filtered_input = prefilter_input
        return filtered_input

    def imageCallback(self, data):
        self.state_image_ = data

    def stateCallback(self): 
        self.state_measured_ = data

    def inputCallback(self, inputDes):
        self.inputAct = self.filterInput(inputDes) 
        self.pub_inputAct_.publish(self.inputAct)


def run_filter():
    safety_filter = measurement_safety_filter()

    
    
    rospy.init_node('measurement_safety_filter', anonymous=True)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown(): 
        #pub_inputAct_.publish(inputAct_)
        #pub_info_.publish(filter_info_)
        rate.sleep()

if __name__ == "__main__": 
    try: 
        run_filter()
    except rospy.ROSInterruptException: 
        pass
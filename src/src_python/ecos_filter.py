#!/usr/bin.env python 

import rospy 
from cyberpod_sim_ros import input, state, filterInfo 
import numpy as np
import matplotlib.pyplot as plt 
import scipy as sp 


def filter(): 
    sub_state_image_ = rospy.Subscriber()
    sub_state_true_  = rospy.Subscriber()
    sub_inputDes_    = rospy.Subscriber()

    pub_inputAct_ = rospy.Publisher('input', queue_size=1) 
    pub_info_     = rospy.Publisher('safety_filter_info', queue_size=1)
    
    rospy.init_node('measurement_safety_filter', anonymous=True)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown(): 
        pub_inputAct_.publish(inputAct_)
        pub_info_.publish(filter_info_)

if __name__ == "main": 
    try: 
        filter()
    except rospy.ROSInterroptException: 
        pass
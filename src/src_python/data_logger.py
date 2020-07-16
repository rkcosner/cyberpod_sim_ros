#!/usr/bin/env python
import rospy 
from cyberpod_sim_ros.msg import * 
import numpy as np 

class data_logger_object(): 
    def __init__(self): 
        # Initialize Subscribers and Publishers
        self.sub_state_true_ = rospy.Subscriber('state_true', state, self.stateTrueCallback)
        self.sub_state_measured_ = rospy.Subscriber('state_measured', state, self.stateMeasuredCallback)
        self.sub_inputDes_ = rospy.Subscriber('inputDes', input, self.inputDesCallback) 
        self.sub_inputAct_ = rospy.Subscriber('input', input, self.inputActCallback)
        
        self.pub_data_log_ = rospy.Publisher('output_data', output_data, queue_size=1)

        # Initialize 
        self.data_log_ = output_data()
        self.data_log_.state_measuredVec = np.zeros(7)
        self.data_log_.input_desVec = np.zeros(2)
        self.data_log_.input_actVec = np.zeros(2)

    # Callback Functions
    def stateTrueCallback(self, state_true_msg): 
        self.data_log_.state_trueVec = state_true_msg.stateVec
        
    
    def stateMeasuredCallback(self, state_measured_msg):
        self.data_log_.state_measuredVec = state_measured_msg.stateVec
    
    def inputDesCallback(self, inputDes_msg):
        self.data_log_.input_desVec = inputDes_msg.inputVec

    def inputActCallback(self, inputAct_msg):
        self.data_log_.input_actVec = inputAct_msg.inputVec
        
        # Finish and Publish Log 
        self.data_log_.time_stamp = rospy.get_rostime()
        self.data_log_.status = inputAct_msg.status
        self.pub_data_log_.publish(self.data_log_)

    

def log_data(): 
    data_logger = data_logger_object()
    
    rospy.init_node('data_logger', anonymous=True)
    rate = rospy.Rate(10) 
    
    rospy.spin()

if __name__ == '__main__': 
    try: 
        log_data()
    except rospy.ROSInterruptException: 
        pass
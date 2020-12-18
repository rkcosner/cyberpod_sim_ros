#!/usr/bin/env python

import rospy 
import numpy as np
import scipy as sp 
import ecos 
from cyberpod_sim_ros.msg import * 
from dynamics import dynamics


class measurementRobustFilter: 
    X5_EQ = 0.138324423615

    def __init__(self):
        self.inputDes_ = input()
        self.inputAct_ = input()
        self.state_image_ = state()
        self.state_measured_ = state()

        # Retrieve Parameters
        self.filter_status_ = rospy.get_param("/cyberpod_sim_ros/mrop_filter/filter")
        self.epsilon_ = float(rospy.get_param("/cyberpod_sim_ros/mrop_filter/epsilon"))
        self.c_ =  float(rospy.get_param("/cyberpod_sim_ros/mrop_filter/c_safety_window"))
        self.alpha_ = rospy.get_param("/cyberpod_sim_ros/mrop_filter/alpha") 
        self.L_lgh_ = rospy.get_param("/cyberpod_sim_ros/mrop_filter/L_lgh") 
        self.L_lfh_ = rospy.get_param("/cyberpod_sim_ros/mrop_filter/L_lfh") 
        self.L_ah_  = rospy.get_param("/cyberpod_sim_ros/mrop_filter/L_ah") 
        self.alpha_e_ = rospy.get_param("/cyberpod_sim_ros/mrop_filter/alpha_e")
        self.D = rospy.get_param("/cyberpod_sim_ros/mrop_filter/D")
        
        # Display Parameters
        rospy.loginfo("Filter node successfully started with:")
        rospy.loginfo("___Filter status=%s" % (self.filter_status_))
        rospy.loginfo("___epsilon=%f" % (self.epsilon_))
        rospy.loginfo("___c=%f" % (self.c_))
        rospy.loginfo("___alpha=%f" % (self.alpha_))
        rospy.loginfo("___L_lfh=%f" % (self.L_lfh_))
        rospy.loginfo("___L_lgh=%f" % (self.L_lgh_))
        rospy.loginfo("___L_ah=%f" % (self.L_ah_))
        rospy.loginfo("___alpha_e=%f" % (self.alpha_e_))

        # Initialize inputDes_ 
        self.inputDes_.inputVec = [0,0] 

        # Init subscribers and publishers
        self.sub_state_measured_  = rospy.Subscriber('state_measured', state, self.stateCallback )
        self.sub_inputDes_    = rospy.Subscriber('inputDes', input, self.inputCallback )
        self.pub_inputAct_ = rospy.Publisher('input', input, queue_size=1) 


    # Callback Functions 
    def stateCallback(self, state_measurement_msg):
        self.state_measured_ = state_measurement_msg

    def inputCallback(self, input_msg):
        if self.filter_status_ =="on":
            self.inputDes_ = input_msg
            self.filterInput()
        else: # filter set to off
            self.inputAct_ = input_msg
        self.pub_inputAct_.publish(self.inputAct_)
    
    # Filter Functions
    def filterInput(self): 
        """
        Filter desired inputs (self.inputDes_) to insure safety as defined by: 
            Safety: 
                h_above = -x[6] + alpha_e*(c - x[5] + X5_EQ), ECBF extension for the safe set x[5] - X5_EQ <= c
                h_below =  x[6] + alpha_e*(c + x[5] - X5_EQ), ECBF extension for the safe set -c <= x[5] - X5_EQ
        The resulting safe inputs are stored in self.inputAct_

        xCurrent = [x, y, yaw, v, yaw_rate, pitch, pitch_rate]
        """

        xCurrent = self.state_measured_.stateVec
        f,g = dynamics(xCurrent)

        # Constraint values for h_above and h_below  
        
        Lfh_wall = -f[3] + self.alpha_e_*(-xCurrent[3])
        Lgh_wall = -g[3] 
        csf_wall = -xCurrent[3] + self.alpha_e_*(self.D - xCurrent[0]) 
        
        # ECOS solve the problem:
        #   min  c'*x
        #   s.t. G*x <=_K h
        # For this problem, K is the product of a 4, 3, and 3 dimensional cones
        # ECOS documentation can be found here: https://github.com/embotech/ecos
        G  = sp.sparse.csc_matrix(
            [[-1/np.sqrt(2), 0, 0, 0], 
            [-1/np.sqrt(2), 0, 0, 0 ], 
            [0, 0, -1, 0], 
            [0, 0, 0, -1], 
            [0, 0, -Lgh_wall[0],  -Lgh_wall[1]],
            [0, 0, -self.epsilon_*self.L_lgh_, 0], 
            [0, 0, 0, -self.epsilon_*self.L_lgh_]]
            )
        h  = np.array(
            [1/np.sqrt(2), 
            -1/np.sqrt(2), 
            0, 
            0, 
            Lfh_wall[0] + self.alpha_*csf_wall,
            0, 
            0, 
            ])
        cost = np.array([1.0, 0.0, -self.inputDes_.inputVec[0],  -self.inputDes_.inputVec[1]])
        dims = {
            'l': 0,         # Dimension of Positive Orthant Cone
            'q': [4,3],     # Dimensions of Second Order Cones
            'e': 0          # Dimension of Exponential Cone
            }

        ecos_solver_output = ecos.solve(cost,G,h,dims, verbose=False) 
        
        test_runtime = False
        if test_runtime: 
            self.inputAct_ = self.inputDes_
        else: 
            if ecos_solver_output['info']['exitFlag'] ==0 or ecos_solver_output['info']['exitFlag'] ==10: # ECOS Solver Successful
                    #rospy.loginfo( (self.inputDes_.inputVec))
                    self.inputAct_ = self.inputDes_
                    self.inputAct_.inputVec = ecos_solver_output['x'][2:4]
                    #rospy.loginfo(self.inputAct_.inputVec)
                    #rospy.loginfo(Lfh_above + np.matmul(Lgh_above,self.inputAct_.inputVec) + self.alpha_*csf_above)
                    #rospy.loginfo('')
            else: # ECOS Solver Failed 
                rospy.logwarn('SOCP failed') # Filter falls back to previous self.inputAct_


    

if __name__ == "__main__": 
    try: 
        rospy.init_node('mrop_filter', anonymous=True)
        safety_filter = measurementRobustFilter()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
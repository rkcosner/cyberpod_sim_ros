#!/usr/bin/env python

import rospy 
from cyberpod_sim_ros.msg import * 
import numpy as np
import scipy as sp 
import ecos 
from dynamics import dynamics

class measurement_safety_filter: 
    


    def __init__(self):
        self.inputDes_ = input()
        self.inputAct_ = input()
        self.state_image_ = state()
        self.state_measured_ = state()

        # Retrieve Parameters
        self.filter_status_ = rospy.get_param("/cyberpod_sim_ros/ecos_safety_filter/filter")
        self.epsilon_ = float(rospy.get_param("/cyberpod_sim_ros/ecos_safety_filter/epsilon"))
        self.C_ =  float(rospy.get_param("/cyberpod_sim_ros/ecos_safety_filter/C"))
        self.alpha_ = rospy.get_param("/cyberpod_sim_ros/ecos_safety_filter/alpha") 
        self.L_lgh_ = rospy.get_param("/cyberpod_sim_ros/ecos_safety_filter/L_lgh") 
        self.L_lfh_ = rospy.get_param("/cyberpod_sim_ros/ecos_safety_filter/L_lfh") 
        self.L_ah_  = rospy.get_param("/cyberpod_sim_ros/ecos_safety_filter/L_ah") 

        rospy.loginfo("Filter node successfully started with:")
        rospy.loginfo("___Filter status=%s" % (self.filter_status_))
        rospy.loginfo("___epsilon=%f" % (self.epsilon_))
        rospy.loginfo("___C=%f" % (self.C_))
        rospy.loginfo("___alpha=%f" % (self.alpha_))
        rospy.loginfo("___L_lfh=%f" % (self.L_lfh_))
        rospy.loginfo("___L_lgh=%f" % (self.L_lgh_))
        rospy.loginfo("___L_ah=%f" % (self.L_ah_))

        # Initialize inputDes_ 
        self.inputDes_.inputVec = [0,0] # dummy value

        # Init pubs, subs
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
        else: # filter off
            self.inputAct_ = input_msg

        self.pub_inputAct_.publish(self.inputAct_)

    # Filter Functions
    def filterInput(self): 
        
        # Safety: 
        #   h = -(1/2)*(x[5]- x5_eq)^2 - (1/2)*x[6]^2 + C 
        #   Safe Set: x[5]^2 + x[6]^2 <= 2*C 
        #   Lfh(x) = -x[6]*x[5] - x[6]*f[6]  
        #   Lgh(x) = -x[6]*g[6,[0,1]]
        #   a(h(x))= alpha*h(x)
        xCurrent = self.state_measured_.stateVec
        f,g = dynamics(xCurrent)
        x5_eq = 0.138
        Lfh = -xCurrent[6]*(xCurrent[5] - x5_eq + f[6])
        Lgh = -xCurrent[6]*g[6,[0,1]]  
        csf = -1/2*((xCurrent[5]-x5_eq)**2 + xCurrent[6]**2) + self.C_
        
        # ECOS solver: 
        #   min  c'*x
        #   s.t. A*x = b
        #        G*x <=_K h
        # For this problem, K is the product of a 4 and a 3 dimensional cone
        G  = [  [-1/np.sqrt(2), 0, 0, 0], 
                [-1/np.sqrt(2), 0, 0, 0 ], 
                [0, 0, -1, 0], 
                [0, 0, 0, -1], 
                [0, 0, -Lgh[0],  -Lgh[1]],
                [0, 0, -self.epsilon_*self.L_lgh_, 0], 
                [0, 0, 0, -self.epsilon_*self.L_lgh_]]
        G  = sp.sparse.csc_matrix(G)
        h  = np.array([  1/np.sqrt(2), 
                        -1/np.sqrt(2), 
                                    0, 
                                    0, 
                        Lfh[0] + self.alpha_*(csf) - (self.L_lfh_ + self.L_ah_)*self.epsilon_, 
                                    0, 
                                    0 ])
        cost = np.array([1.0, 0.0, -self.inputDes_.inputVec[0],  -self.inputDes_.inputVec[1]])
        dims = {
            'l': 0,     # Dimension of Positive Orthant Cone
            'q': [4,3], # Dimensions of Second Order Cones
            'e': 0      # Dimension of Exponential Cone
        }
        output = ecos.solve(cost,G,h,dims, verbose=False) 

        if output['info']['exitFlag'] ==0 or output['info']['exitFlag'] ==10 or output['info']['exitFlag'] ==-1: # Global Optimal Found
            self.inputAct_ = self.inputDes_
            self.inputAct_.inputVec = output['x'][2:4]
        else:
            # repeat previous input 
            rospy.logwarn('SOCP failed')

    def dontFilterInput(self): 
        self.inputAct_ = self.inputDes_


def run_filter():
    rospy.init_node('measurement_safety_filter', anonymous=True)
    safety_filter = measurement_safety_filter()

    rate = rospy.Rate(10)     
    rospy.spin()

if __name__ == "__main__": 
    try: 
        run_filter()
    except rospy.ROSInterruptException: 
        pass
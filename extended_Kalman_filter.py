'''
extended_Kalman_filter.py
Implementation of the extended Kalman Filter functions.
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''
import numpy as np
import math


class Kalman():
    def __init__(self):
        '''
        Initialization of the Kalman class
        '''
        # state noise variances (experimental values)
        self.q_x     = 0
        self.q_y     = 0
        self.q_theta = 0
        self.q_vr    = 0    # we assume that the speed variance is half caused by measurement
        self.q_vl    = 0    # and half by perturbation as seen in the exercise 8
        
        # state matrix (the A matrix and the motion model Jacobian G have to be computed at each iteration)
        self.B = ([0, 0],
                  [0, 0],
                  [0, 0],
                  [1, 0],
                  [0, 1])
        
        # state noise matrices (we assume that the noises are independent for simplicity)
        self.Q_nc = np.diag([self.q_vr],    # no camera
                            [self.q_vl])
        self.Q_wc = np.diag([self.q_x],     # with camera
                            [self.q_y],
                            [self.q_theta],
                            [self.q_vr], 
                            [self.q_vl])

        # measurement noise variances (experimental values)
        self.r_x     = 0
        self.r_y     = 0
        self.r_theta = 0
        self.r_vr    = self.q_vr    # we assume that the speed variance is half caused by measurement 
        self.r_vl    = self.q_vl    # and half by perturbation as seen in the exercise 8

        # measurements matrices
        self.H_nc = ([0, 0, 0, 1, 0],   # no camera
                     [0, 0, 0, 0, 1])
        self.H_wc = np.diag(np.ones(5)) # with camera

        # measurements noise matrices (we assume that the noises are independent for simplicity)
        self.Q_nc = np.diag([self.r_vr],    # no camera
                            [self.r_vl])
        self.Q_wc = np.diag([self.r_x],     # with camera
                            [self.r_y],
                            [self.r_theta],
                            [self.r_vr], 
                            [self.r_vl])
        

        


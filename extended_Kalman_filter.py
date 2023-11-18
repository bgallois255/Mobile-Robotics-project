'''
extended_Kalman_filter.py
Implementation of the extended Kalman Filter functions.
Authors: Benoît Gallois, Jehan Corcelle, Arto Dubuisson, Raphaël Dousson
'''
import numpy as np
import math

import constantes as cst

class Kalman():
    def __init__(self, x, y, theta):
        '''
        Initialization of the Kalman class
        '''
        # initial position given by the camera
        self.x = x
        self.y = y
        self.theta = theta
        self.vr = 0     # at initialization the robot is not moving
        self.vl = 0

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
        self.Q_nc = np.diag([self.q_vr,    # no camera
                            self.q_vl])
        self.Q_wc = np.diag([self.q_x,     # with camera
                            self.q_y,
                            self.q_theta,
                            self.q_vr, 
                            self.q_vl])

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
        self.R_nc = np.diag([self.r_vr,    # no camera
                            self.r_vl])
        self.R_wc = np.diag([self.r_x,     # with camera
                            self.r_y,
                            self.r_theta,
                            self.r_vr, 
                            self.r_vl])
        
    def matrices_update(self, dt, theta, vr, vl):
        '''
        System evolving matrix A matrix and motion model Jacobian G computation
        (needs to be done at each iteration)

        Inputs: - dt    : delta time, time between two iterations in [s]
                - theta : robot angle in [rad]
                - vr    : right wheel velocity in [m/2]
                - vl    : left wheel velocity in [m/2]
        Outputs:- A     : updated system evolving matrix
                - G     : updated motion model Jacobian 
    
        '''
        A = ([1, 0, 0, dt*math.cos(theta)/2  , dt*math.cos(theta)/2],
             [0, 1, 0, dt*math.sin(theta)/2  , dt*math.sin(theta)/2],
             [0, 0, 1, dt/(4*cst.WHEELS_DIST), -dt/(4*cst.WHEELS_DIST)],
             [0, 0, 0, 0                     , 0],
             [0, 0, 0, 0                     , 0])
        
        G = A + np.diag([0,0,0,1,1])
        G[0, 2] = dt*math.sin(theta)*(vr+vl)/2
        G[1, 2] = -dt*math.cos(theta)*(vr+vl)/2

        return A,G
        


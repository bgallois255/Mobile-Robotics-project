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

        Inputs: - x     : initial x position given by the camera in [m]
                - y     : initial y position given by the camera in [m]
                - theta : initial angle given by the camera in [rad]
        '''
        # state noise variances (experimental values)
        self.q_x     = 0
        self.q_y     = 0
        self.q_theta = 0
        self.q_vr    = 0    # we assume that the speed variance is half caused by measurement
        self.q_vl    = 0    # and half by perturbation as seen in the exercise 8

        # state vector, initial position and angle given by the camera
        self.Mu = ([[x],     # x position
                   [y],      # y position
                   [theta],  # angle
                   [0],      # vr, right wheel velocity: at initialization the robot is not moving
                   [0]])     # vl, left wheel velocity : at initialization the robot is not moving

        # covariance matrix of state, with initial position noise variance
        self.Sigma = np.diag([self.q_x,
                              self.q_y,
                              self.q_theta,
                              0,              # at initialization the robot is not moving -> no variance
                              0])             # at initialization the robot is not moving -> no variance
        
        # state matrix (the A matrix and the motion model Jacobian G have to be computed at each iteration)
        self.B = ([[0, 0],
                  [0, 0],
                  [0, 0],
                  [1, 0],
                  [0, 1]])
        
        # state noise matrices (we assume that the noises are independent for simplicity)
        self.Q = np.diag([self.q_x,
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
        self.H_nc = ([[0, 0, 0, 1, 0],   # no camera
                     [0, 0, 0, 0, 1]])
        self.H_wc = np.diag(np.ones(5)) # with camera

        # measurements noise matrices (we assume that the noises are independent for simplicity)
        self.R_nc = np.diag([self.r_vr,    # no camera
                            self.r_vl])
        self.R_wc = np.diag([self.r_x,     # with camera
                            self.r_y,
                            self.r_theta,
                            self.r_vr, 
                            self.r_vl])
        
    def matrices_update(self, dt, vr, vl):
        '''
        System evolving matrix A matrix and motion model Jacobian G computation
        (needs to be done at each iteration)

        Inputs: - dt    : delta time, time between two iterations in [s]
                - vr    : input right wheel velocity in [m/2]
                - vl    : input left wheel velocity in [m/2]
        Outputs:- A     : updated system evolving matrix
                - G     : updated motion model Jacobian 
        '''
        theta = self.Mu[2][0]

        A = ([[1, 0, 0, dt*math.cos(theta)/2  , dt*math.cos(theta)/2],
             [0, 1, 0, dt*math.sin(theta)/2  , dt*math.sin(theta)/2],
             [0, 0, 1, dt/(4*cst.WHEELS_DIST), -dt/(4*cst.WHEELS_DIST)],
             [0, 0, 0, 0                     , 0],
             [0, 0, 0, 0                     , 0]])
        
        G = A + np.diag([0,0,0,1,1])
        G[0][2] = dt*math.sin(theta)*(vr+vl)/2
        G[1][2] = -dt*math.cos(theta)*(vr+vl)/2

        return A,G
        
    def Kalman_filter(self, dt, vr, vl):
        '''
        Kalman filter implementation, system state and variance update

        Inputs: - dt    : delta time, time between two iterations in [s]
                - vr    : input right wheel velocity in [m/2]
                - vl    : input left wheel velocity in [m/2]
        Outputs:-      : 
                -      :  
        '''
        input = ([0], [0], [0], [vr], [vl])
        A,G = self.matrices_update(self, dt, vr, vl)

        # State (Mu) prediction
        Mu_bar = np.dot(A, self.Mu) + np.dot(self.B, input)
        # Variance (Sigma) prediction
        Sigma_bar = np.dot(G, np.dot(self.Sigma, G.T)) + self.Q
        # innovation
        






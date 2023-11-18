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

        Return: - x     : initial x position given by the camera in [m]
                - y     : initial y position given by the camera in [m]
                - theta : initial angle given by the camera in [rad]
        '''
        # time interval between two iterations
        self.dt = 0

        # state vector, initial position and angle given by the camera
        self.Mu = ([[x],     # x position
                   [y],      # y position
                   [theta],  # angle
                   [0],      # vr, right wheel velocity: at initialization the robot is not moving
                   [0]])     # vl, left wheel velocity : at initialization the robot is not moving

        # state noise variances (experimental values)
        self.q_x     = 0
        self.q_y     = 0
        self.q_theta = 0
        self.q_vr    = 0    # we assume that the speed variance is half caused by measurement
        self.q_vl    = 0    # and half by perturbation as seen in the exercise 8

        # covariance matrix of state, with initial position noise variance
        self.Sigma = np.diag([self.q_x,
                              self.q_y,
                              self.q_theta,
                              0,              # at initialization the robot is not moving -> no variance
                              0])             # at initialization the robot is not moving -> no variance
        self.Sigma += np.eye(5)*0.00001       # add small number to be able to inverse Sigma even if one element of its diagonal is 0

        # state matrix (the A matrix and the motion model Jacobian G have to be computed at each iteration)
        self.B = ([[0, 0],
                  [0, 0],
                  [0, 0],
                  [1, 0],
                  [0, 1]])
        
        # state noise matrices (we assume that the noises are independent for simplicity)
        self.Q = np.diag([self.q_x, self.q_y, self.q_theta, self.q_vr, self.q_vl])

        # measurements vector, initial position and angle given by the camera
        self.y = ([[x],      # x position
                   [y],      # y position
                   [theta],  # angle
                   [0],      # vr, right wheel velocity: at initialization the robot is not moving
                   [0]])     # vl, left wheel velocity : at initialization the robot is not moving
        
        # measurement noise variances (experimental values)
        self.r_x     = 0
        self.r_y     = 0
        self.r_theta = 0
        self.r_vr    = self.q_vr    # we assume that the speed variance is half caused by measurement 
        self.r_vl    = self.q_vl    # and half by perturbation as seen in the exercise 8


    def dt_update(self, dt):
        '''
        update of the time interval between two iterations

        Parameter: - dt : time interval between two iterations
        '''
        self.dt = dt

    def measurement_wheels(self, vr, vl):
        '''
        update of the measurement of the wheels speed

        Parameter: - vr    : right wheel velocity in [m/2]
                   - vl    : left wheel velocity in [m/2]
        '''
        self.y[3][0] = vr
        self.y[4][0] = vl

    def measurement_position(self, x, y, theta):
        '''
        update of the measurement of the position

        Parameter: - x     : x position in [m]
                   - y     : y position in [m]
                   - theta : angle in [rad]
        '''
        self.y[0][0] = x
        self.y[1][0] = y
        self.y[2][0] = theta

    def model_update(self):
        '''
        System evolving matrix A matrix and motion model Jacobian G computation
        (needs to be done at each iteration)

        Return: - A     : updated system evolving matrix
                - G     : updated motion model Jacobian 
        '''
        theta = self.Mu[2][0]       # angle in [rad]
        vr = self.Mu[3][0]          # right wheel velocity in [m/2]
        vl = self.Mu[4][0]          # left  wheel velocity in [m/2]

        A = ([[1, 0, 0, self.dt*math.cos(theta)/2  , self.dt*math.cos(theta)/2],
              [0, 1, 0, self.dt*math.sin(theta)/2  , self.dt*math.sin(theta)/2],
              [0, 0, 1, self.dt/(4*cst.WHEELS_DIST), -self.dt/(4*cst.WHEELS_DIST)],
              [0, 0, 0, 0                          , 0],
              [0, 0, 0, 0                          , 0]])
        
        G = A + np.diag([0,0,0,1,1])
        G[0][2] = self.dt*math.sin(theta)*(vr+vl)/2
        G[1][2] = -self.dt*math.cos(theta)*(vr+vl)/2

        return A,G
        
    def Kalman_filter(self):
        '''
        Kalman filter implementation, system state and variance update

        Parameter: - vr    : input right wheel velocity in [m/2]
                   - vl    : input left wheel velocity in [m/2]
                   - x     : input x position given by the camera in [m] (= None if not provided)
                   - y     : input y position given by the camera in [m] (= None if not provided)
                   - theta : input angle given by the camera in [rad]    (= None if not provided)

        Return: -      : 
                -      :  
        '''
        A,G = self.model_update(self)                               # update model matrices

        Mu_bar = np.dot(A, self.Mu) + np.dot(self.B, input)         # State (Mu) prediction
        Sigma_bar = np.dot(G, np.dot(self.Sigma, G.T)) + self.Q     # Variance (Sigma) prediction 

        if(self.Mu[0][0] is None):                                  # measurements without camera (only wheels speed)
            measurement = ([self.y[3][0]],
                           [self.y[4][0]])
            H = ([[0, 0, 0, 1, 0],                                  # measurement model matrix
                  [0, 0, 0, 0, 1]])
            R = np.diag([self.r_vr, self.r_vl])     # measurements noise matrix (we assume that the noises are independent for simplicity)
        
        else:                                                       # measurements with camera (position and wheels speed)
            measurement = self.y
            H = np.eye(5)                                           # measurement model matrix
            R = np.diag([self.r_x, self.r_y, self.r_theta, self.r_vr, self.r_vl])   # measurements noise matrix
        
        innovation = measurement - np.dot(H, Mu_bar)
        S = np.dot(H, np.dot(Sigma_bar, H.T)) + R                   # innovation variance
        K = np.dot(Sigma_bar, np.dot(H.T, np.linalg.inv(S)))        # optimal gain
        self.Mu = Mu_bar + np.dot(K, innovation)                    # update state
        self.Sigma = np.dot((np.eye(5) - np.dot(K, H)), Sigma_bar)  # update variance state
        self.Sigma += np.eye(5)*0.00001             # add small number to be able to inverse Sigma even if one element of its diagonal is 0
        
        







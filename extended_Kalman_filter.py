'''
File : extended_Kalman_filter.py 
Author : Raphaël Dousson
Date : 6 déc 2023
Implementation of the extended Kalman Filter functions.
'''

#==================================================================
import numpy as np
import math
#==================================================================

WHEELS_DIST = 93                               # distance between the wheels of Thymio in [mm]
ROTATION_CORR_LEFT = 25                        # correction factor for rotations
ROTATION_CORR_RIGHT = 50                       # correction factor for rotations
TRANSLATION_CORR = 0.35

#==================================================================

class Kalman():
    def __init__(self, x, y, theta):
        '''
        @brief   Detects areas corresponding to a color and returns the coordinates of the vertices and add a margin.

        @param   x      -> initial x position given by the camera in [mm]
                 y      -> initial y position given by the camera in [mm]
                 theta  -> initial angle given by the camera in [rad]
        '''
        
        # time interval between two iterations
        self.dt = 0.0

        # state vector, initial position and angle given by the camera
        self.Mu = np.array([[x],     # x position
                            [y],      # y position
                            [theta],  # angle
                            [0.0],      # vr, right wheel velocity: at initialization the robot is not moving
                            [0.0]])     # vl, left wheel velocity : at initialization the robot is not moving

        # state noise variances (experimental values)
        self.q_x     = 0.001
        self.q_y     = 0.001
        self.q_theta = 0.001
        self.q_vr    = 8.318/2.0    # we assume that the speed variance is half caused by measurement
        self.q_vl    = 8.318/2.0    # and half by perturbation as seen in the exercise 8

        # covariance matrix of state, with initial position noise variance
        self.Sigma = np.diag([self.q_x,
                              self.q_y,
                              self.q_theta,
                              0.0,             # at initialization the robot is not moving -> no variance
                              0.0])            # at initialization the robot is not moving -> no variance
        self.Sigma += np.eye(5)*0.00001        # add small number to be able to inverse Sigma even if one element of its                                                        diagonal is 0

        # state matrix (the A matrix and the motion model Jacobian G have to be computed at each iteration)
        self.B = np.array([[0, 0],
                           [0, 0],
                           [0, 0],
                           [1, 0],
                           [0, 1]])
        
        # state noise matrices (we assume that the noises are independent for simplicity)
        self.Q = np.diag([self.q_x, self.q_y, self.q_theta, self.q_vr, self.q_vl])

        # measurements vector, initial position and angle given by the camera
        self.y = np.array([[x],        # x position
                           [y],        # y position
                           [theta],    # angle
                           [0.0],      # vr, right wheel velocity: at initialization the robot is not moving
                           [0.0]])     # vl, left wheel velocity : at initialization the robot is not moving
        
        # information if position given by the camera or not
        self.position_camera = True    # at initialization the camera gives the robot position
        
        # measurement noise variances (experimental values)
        self.r_x     = 0.001
        self.r_y     = 0.001
        self.r_theta = 0.001
        self.r_vr    = self.q_vr    # we assume that the speed variance is half caused by measurement 
        self.r_vl    = self.q_vl    # and half by perturbation as seen in the exercise 8


    def dt_update(self, dt):
        '''
        @brief   Updates the time interval between two iterations.

        @param   dt  -> Time interval between two iterations
        '''
        self.dt = dt
        

    def measurement_wheels(self, vr, vl):
        '''
        @brief   Updates the measurement of the wheels speed

        @param   vr    -> right wheel velocity in [mm/s]
                 vl    -> left wheel velocity in [mm/s]
        '''
        self.y[3][0] = vr
        self.y[4][0] = vl
        

    def measurement_position(self, x, y, theta):
        '''
        @brief   Updates the measurement of the position

        @param   x       -> x position in [mm]
                 y       -> y position in [mm]
                 theta   -> angle in [rad]
        '''
        self.y[0][0] = x
        self.y[1][0] = y
        self.y[2][0] = theta
        self.position_camera = True

        
    def model_update(self):
        '''
        @brief   System evolving matrix A matrix and motion model Jacobian G computation (needs to be done at each iteration).

        @return  A    -> updated system evolving matrix
                 G    -> updated motion model Jacobian 
        '''
        
        theta = self.Mu[2][0]       # angle in [rad]
        vr = self.Mu[3][0]          # right wheel velocity in [mm/s]
        vl = self.Mu[4][0]          # left  wheel velocity in [mm/s]

        A = np.array([[1, 0, 0, self.dt*TRANSLATION_CORR*math.cos(theta)/2  , self.dt*TRANSLATION_CORR*math.cos(theta)/2   ],
                      [0, 1, 0, self.dt*TRANSLATION_CORR*math.sin(theta)/2  , self.dt*TRANSLATION_CORR*math.sin(theta)/2   ],
                      [0, 0, 1, self.dt*ROTATION_CORR_RIGHT/(4*WHEELS_DIST), -self.dt*ROTATION_CORR_LEFT/(4*WHEELS_DIST)], 
                      [0, 0, 0, 0                          , 0                           ],
                      [0, 0, 0, 0                          , 0                           ]])
        
        G = A + np.diag([0,0,0,1,1])
        G[0][2] = self.dt*TRANSLATION_CORR*math.sin(theta)*(vr+vl)/2
        G[1][2] = -self.dt*TRANSLATION_CORR*math.cos(theta)*(vr+vl)/2

        return A,G
   
        
    def Kalman_filter(self):
        '''
        @brief   Kalman filter implementation, system state and variance update

        @return  vr     -> input right wheel velocity in [mm/s]
                 vl     -> input left wheel velocity in [mm/s]
                 x      -> input x position  given by the camera in [mm]
                 y      -> input y position given by the camera in [mm]
                 theta  -> input angle given by the camera in [rad]
        '''
        
        A,G = self.model_update()                                   # update model matrices
        input = np.array([[self.Mu[3][0]],                          # input = vr, vl
                          [self.Mu[4][0]]])

        Mu_bar = np.dot(A, self.Mu) + np.dot(self.B, input)         # State (Mu) prediction

        Sigma_bar = np.dot(G, np.dot(self.Sigma, G.T)) + self.Q     # Variance (Sigma) prediction 

        if(not self.position_camera):                               # measurements without camera (only wheels speed)
            measurement = np.array([[self.y[3][0]],
                                    [self.y[4][0]]])
            H = np.array([[0, 0, 0, 1, 0],                          # measurement model matrix
                          [0, 0, 0, 0, 1]])                                               
            R = np.diag([self.r_vr, self.r_vl])                     # measurements noise matrix (we assume that the noises are                                                                       independent for simplicity)
      
        else:                                                       # measurements with camera (position and wheels speed)
            measurement = self.y
            H = np.eye(5)                                           # measurement model matrix
            R = np.diag([self.r_x, self.r_y, self.r_theta, self.r_vr, self.r_vl])   # measurements noise matrix

        innovation = measurement - np.dot(H, Mu_bar)
        S = np.dot(H, np.dot(Sigma_bar, H.T)) + R                   # innovation variance
        K = np.dot(Sigma_bar, np.dot(H.T, np.linalg.inv(S)))        # optimal gain
        self.Mu = Mu_bar + np.dot(K, innovation)                    # update state
        self.Sigma = np.dot((np.eye(5) - np.dot(K, H)), Sigma_bar)  # update variance state
        self.Sigma += np.eye(5)*0.00001                             # add small number to be able to inverse Sigma even if one                                                                       element of its diagonal is 0
        self.position_camera = False                                # reset camera reading


def center_angle(angle):
    '''
    @brief   Compute the equivalent angle between -pi and pi.

    @param   angle  -> angle to be centered in [rad]
    
    @return  angle  -> centered angle in [rad]
    '''
    
    while(angle > math.pi or angle <= - math.pi):
        if(angle > math.pi):
            angle -= 2*math.pi
        elif(angle <= - math.pi):
            angle += 2*math.pi

    return angle





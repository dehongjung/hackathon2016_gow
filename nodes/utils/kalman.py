#!/usr/bin/env python

# Year: 2014 
# Author: EDX Courses, 
#   modified by: Marcela P. Carvalho, email: marcela_pcarvalho@hotmail.com

# Standard imports
#import roslib
#roslib.load_manifest('tg_nao_teleoperation')
#import rospy
import numpy as np

class KalmanFilter:
    def __init__(self):
#        dt = 0.005
        dt = 1/100 # this comes from publish rate
        #State-transition model
        self.A = np.array([
            [1,0,0,dt,0,0],
            [0,1,0,0,dt,0],
            [0,0,1,0,0,dt],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1],
        ]) 
        #Observation model
        self.H = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
        ]) 
        
        #TODO: Play with the noise matrices
        #Process/State noise
        posx_noise_std = 0.005
        posy_noise_std = 0.005
        posz_noise_std = 0.005
        velx_noise_std = 0.005
        vely_noise_std = 0.005
        velz_noise_std = 0.005
        self.Q = np.array([
            [posx_noise_std*posx_noise_std,0,0,0,0,0],
            [0,posy_noise_std*posy_noise_std,0,0,0,0],
            [0,0,posy_noise_std*posz_noise_std,0,0,0],
            [0,0,0,velx_noise_std*velx_noise_std,0,0],
            [0,0,0,0,vely_noise_std*vely_noise_std,0],
            [0,0,0,0,0,velz_noise_std*velz_noise_std]
        ]) 
        
        #Sensor/Measurement noise
        measurement_noise_std = 0.5#0.08
        self.R = measurement_noise_std * measurement_noise_std * measurement_noise_std *np.identity(3) 

        self.x = np.zeros((6,1)) #Initial state vector [x,y,z,vx,vy,vz]
        self.sigma = np.identity(6) #Initial covariance matrix
    
    def predictState(self, A, x):
        '''
        :param A: State-transition model matrix
        :param x: Current state vector
        :return x_p: Predicted state vector as 4x1 numpy array
        '''
        
        #TODO: Predict the next state
        x_p = np.dot(A,x)
        
        return x_p
    
    def predictCovariance(self, A, sigma, Q):
        sigma_p = np.dot(np.dot(A, sigma), np.transpose(A))+Q 
        return sigma_p
    
    def calculateKalmanGain(self, sigma_p, H, R):
        k = np.dot(np.dot(sigma_p, np.transpose(H)), np.linalg.inv(np.dot(H, np.dot(sigma_p, np.transpose(H)))+R))
        return k
    
    def correctState(self, z, x_p, k, H):
        '''
        :param z: Measurement vector
        :param x_p: Predicted state vector
        :param k: Kalman gain
        :param H: Observation model
        :return x: Corrected state vector as 6x1 numpy array
        '''
        
        #TODO: Correct the current state prediction with the measurement
        x = x_p + np.dot(k,z-np.dot(H,x_p))

        return x
    
    def correctCovariance(self, sigma_p, k, H):
        sigma = np.dot((np.identity(6)-np.dot(k, H)), sigma_p)
        return sigma
    
    def state_callback(self):
#        self.A = self.redefine
    
        self.x = self.predictState(self.A, self.x)
        self.sigma = self.predictCovariance(self.A, self.sigma, self.Q)
        
    def measurement_callback(self, measurement):
        '''
        :param measurement: vector of measured coordinates
        '''
        
        
        k = self.calculateKalmanGain(self.sigma, self.H, self.R)
        
        self.x = self.correctState(measurement, self.x, k, self.H)
        self.sigma = self.correctCovariance(self.sigma, k, self.H)
        

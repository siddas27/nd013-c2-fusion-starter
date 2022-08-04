# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        pass

    def F(self):
        ############
        # Step 1: implement and return system matrix F
        ############
        hdim = int(params.dim_state / 2)
        F = np.identity(hdim) * params.dt
        F = np.hstack((np.zeros((hdim, hdim)), F))
        F = np.vstack((F, np.zeros((hdim, params.dim_state))))
        
        return np.identity(params.dim_state) + F
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # Step 1: implement and return process noise covariance Q
        ############
        hdim = int(params.dim_state / 2)
        dt = params.dt
        Q1 = np.identity(hdim) *((dt**3)/3)* params.q
        Q2 = np.identity(hdim) *((dt**2)/2)* params.q
        Q3 = np.identity(hdim) *dt* params.q

        Q12 = np.hstack((Q1, Q2))
        Q23 = np.hstack((Q2, Q3))

        Q = np.vstack((Q12,Q23))
        return Q
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        Q=self.Q()
        track.set_x(F*track.x)
        track.set_P(F*track.P*F.transpose()+Q)
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        gamma = self.gamma(track, meas)
        H= meas.sensor.get_H(track.x)
        S = self.S(track,meas,H)
        K= track.P*H.transpose()*np.linalg.inv(S)
        track.x = track.x + K*gamma
        I = np.identity(params.dim_state)
        track.P = (I-K*H)*track.P
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # Step 1: calculate and return residual gamma
        ############
        H = meas.sensor.get_H(track.x)
        return meas.z - H*track.x
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # Step 1: calculate and return covariance of residual S
        ############

        return H*track.P*H.transpose()+meas.R
        
        ############
        # END student code
        ############ 
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
        F = np.eye(6)
        F[0, 3] = params.dt
        F[1, 4] = params.dt
        F[2, 5] = params.dt
        
        return np.matrix(F)

    def Q(self):
        a = 1 / 3 * params.dt ** 3 * params.q
        b = 1 / 2 * params.dt ** 2 * params.q
        c = params.dt * params.q
        return np.matrix([
            [a, 0, 0, b, 0, 0],
            [0, a, 0, 0, b, 0],
            [0, 0, a, 0, 0, b],
            [b, 0, 0, c, 0, 0],
            [0, b, 0, 0, c, 0],
            [0, 0, b, 0, 0, c]
        ])        
        
    def predict(self, track):
        F = self.F()
        Q = self.Q()
        x = F * track.x
        P = F * track.P * F.T + Q
        track.set_x(x)
        track.set_P(P)

    def update(self, track, meas):
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        K = track.P * H.T * np.linalg.inv(S)
        x = track.x + K * self.gamma(track, meas)
        I = np.eye(6)
        P = (I - K * H) * track.P
        
        track.set_x(x)
        track.set_P(P)
        
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        return meas.z - meas.sensor.get_hx(track.x)

    def S(self, track, meas, H):
        return H * track.P * H.T + meas.R
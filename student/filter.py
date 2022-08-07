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
        self._delta_time = params.dt
        self._q = params.q  # process noise for Kalman filter Q
        self._dim_state = params.dim_state

    def F(self):
        ############
        # Step 1: implement and return system matrix F
        ############
        dt = self._delta_time
        # Reference for 6D matrix: https://knowledge.udacity.com/questions/794594
        return np.matrix([[1, 0, 0, dt, 0, 0],  # x
                          [0, 1, 0, 0, dt, 0],  # y
                          [0, 0, 1, 0, 0, dt],  # z
                          [0, 0, 0, 1, 0, 0],   # vx
                          [0, 0, 0, 0, 1, 0],   # vy
                          [0, 0, 0, 0, 0, 1]])  # vz

        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # Step 1: implement and return process noise covariance Q
        ############
        q = self._q
        dt = self._delta_time

        q1 = dt * q
        q2 = ((dt**2) / 2) * q
        q3 = ((dt**3) / 3) * q
        # Reference for 6D matrix: https://knowledge.udacity.com/questions/794594
        return np.matrix([[q3, 0, 0, q2, 0, 0],
                          [0, q3, 0, 0, q2, 0],
                          [0, 0, q3, 0, 0, q2],
                          [q2, 0, 0, q1, 0, 0],
                          [0, q2, 0, 0, q1, 0],
                          [0, 0, q2, 0, 0, q1]])

        ############
        # END student code
        ############

    def predict(self, track):
        ############
        # Step 1: predict state x and estimation error covariance P to next
        #         timestep, save x and P in track
        ############
        F = self.F()
        track.set_x(F * track.x)
        track.set_P(F * track.P * F.transpose() + self.Q())

        ############
        # END student code
        ############

    def update(self, track, meas):
        ############
        # Step 1: update state x and covariance P with associated measurement,
        #         save x and P in track
        ############
        H = meas.sensor.get_H(track.x)  # measurement matrix
        gamma = self.gamma(track, meas)  # residual
        S = self.S(track, meas, H)  # Covariance of residual
        K = track.P * H.transpose() * np.linalg.inv(S)  # Kalman gain
        track.set_x(track.x + (K * gamma))  # Update state
        I = np.identity(self._dim_state)
        track.set_P(I - (K * H) * track.P)  # covariance update

        ############
        # END student code
        ############
        track.update_attributes(meas)

    def gamma(self, track, meas):
        ############
        # Step 1: calculate and return residual gamma
        ############

        return (meas.z - meas.sensor.get_hx(track.x))

        ############
        # END student code
        ############

    def S(self, track, meas, H):
        ############
        # Step 1: calculate and return covariance of residual S
        ############

        return ((H * track.P * H.transpose()) + meas.R)

        ############
        # END student code
        ############

# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Data association class with single nearest neighbor association and gating based on Mahalanobis distance
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
from scipy.stats.distributions import chi2
import misc.params as params

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))


class Association:
    '''Data association class with single nearest neighbor association and gating based on Mahalanobis distance'''
    def __init__(self):
        self.association_matrix = np.matrix([])
        self.unassigned_tracks = []
        self.unassigned_meas = []

    def associate(self, track_list, meas_list, KF):
        ############
        # Step 3: association:
        # - replace association_matrix with the actual association matrix
        #   based on Mahalanobis distance (see below) for all tracks and all
        #   measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        tl_len = len(track_list)
        ml_len = len(meas_list)

        # the following only works for at most one track and one measurement
        self.association_matrix = np.asmatrix(
            np.inf * np.ones((tl_len, ml_len)))
        self.unassigned_tracks = list(range(tl_len))
        self.unassigned_meas = list(range(ml_len))

        for i, track in enumerate(track_list):
            for j, meas in enumerate(meas_list):
                dist = self.MHD(track, meas, KF)
                if self.gating(dist, meas.sensor):
                    self.association_matrix[i, j] = dist

        ############
        # END student code
        ############

    def get_closest_track_and_meas(self):
        ############
        # Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks
        #   and unassigned_meas
        # - return this track and measurement
        # Reference: Gating section of Multi-Target Tracking
        ############
        A = self.association_matrix
        # Verify association matrix doesn't have any values other than inf
        if np.min(A) == np.inf:
            return np.nan, np.nan

        # Get the indices of the min. entry
        idx_trk, idx_meas = np.unravel_index(np.argmin(A, axis=None), A.shape)

        # Delete row and col for next update
        A = np.delete(A, idx_trk, 0)
        A = np.delete(A, idx_meas, 1)

        # the following only works for at most one track and one measurement
        update_track = self.unassigned_tracks[idx_trk]
        update_meas = self.unassigned_meas[idx_meas]

        # remove from list
        self.unassigned_tracks.remove(update_track)
        self.unassigned_meas.remove(update_meas)
        self.association_matrix = A

        ############
        # END student code
        ############
        return update_track, update_meas

    def gating(self, MHD, sensor):
        ############
        # Step 3: return True if measurement lies inside gate, otherwise False
        # Reference: Gating section in Multi-Target Tracking
        ############
        gate = chi2.ppf(params.gating_threshold, df=sensor.dim_meas)
        return (MHD < gate)

        ############
        # END student code
        ############

    def MHD(self, track, meas, KF):
        ############
        # Step 3: calculate and return Mahalanobis distance
        # Reference: Association Matrix section of Multi-Target Tracking
        ############
        H = meas.sensor.get_H(track.x)
        gamma = KF.gamma(track, meas)
        S = KF.S(track, meas, H)
        return gamma.transpose() * S.I * gamma

        ############
        # END student code
        ############

    def associate_and_update(self, manager, meas_list, KF):
        # associate measurements and tracks
        self.associate(manager.track_list, meas_list, KF)

        # update associated tracks with measurements
        while self.association_matrix.shape[0] > 0 and \
              self.association_matrix.shape[1] > 0:
            # search for next association between a track and a measurement
            ind_track, ind_meas = self.get_closest_track_and_meas()
            if np.isnan(ind_track):
                print('---no more associations---')
                break
            track = manager.track_list[ind_track]

            # check visibility, only update tracks in fov
            if not meas_list[0].sensor.in_fov(track.x):
                continue

            # Kalman update
            print('update track', track.id, 'with',
                  meas_list[ind_meas].sensor.name, 'measurement', ind_meas)
            KF.update(track, meas_list[ind_meas])

            # update score and track state
            manager.handle_updated_track(track)

            # save updated track
            manager.track_list[ind_track] = track

        # run track management
        manager.manage_tracks(self.unassigned_tracks,
                              self.unassigned_meas, meas_list)

        for track in manager.track_list:
            print('track', track.id, 'score =', track.score)

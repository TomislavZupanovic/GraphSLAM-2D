#!/usr/bin/env python

import numpy as np

class SLAM2D(object):
    def __init__(self, steps, world_size, num_landmarks, measurement_noise, motion_noise):
        self.steps = steps
        self.world_size = world_size
        self.num_landmarks = num_landmarks
        self.measurement_noise = measurement_noise
        self.motion_noise = motion_noise
        self.omega = None
        self.xi = None
        self.mu = None

    def _initialize_constraints(self):
        """ Initializes Omega and Xi with appropriate size and initial position values """
        matrix_size = (self.steps + self.num_landmarks) * 2
        self.omega = np.zeros((matrix_size, matrix_size))
        self.xi = np.zeros((matrix_size, 1))
        # Set initial position
        self.omega[0][0] = 1
        self.omega[1][1] = 1
        self.xi[0:2] = self.world_size / 2

    def _update_omega(self, index1, index2, noise):
        """ Updates Omega matrix on appropriate indexes with added noise """
        if self.omega is not None:
            self.omega[index1, index1] += 1/noise
            self.omega[index1, index2] -= 1/noise
            self.omega[index2, index1] -= 1/noise
            self.omega[index2, index2] += 1/noise
        else:
            raise ValueError('Omega not initialized.')

    def _update_xi(self, index1, index2, value, noise):
        """ Updates Xi vector on appropriate indexes with added noise """
        if self.xi is not None:
            self.xi[index1] -= value/noise
            self.xi[index2] += value/noise
        else:
            raise ValueError('Xi not initialized.')

    def run_slam(self, data):
        """ Runs the SLAM algorithm with measurement and motion updates, calculates the mu vector
            with all estimated Robot movement and landmark locations """
        print('\nRunning SLAM...')
        # Initialize Omega and Xi with start position
        self._initialize_constraints()
        # Get all the motion and measurement iterating over data
        for step in range(len(data)):
            # Get measurement and motion for given step
            measurements = data[step][0]
            motion = data[step][1]
            # Repeat process for x and y dimensions
            for dim in [0, 1]:
                # Calculate indexes based on current step and dimension
                index = 2 * step + dim
                index_2 = index + 2
                # Measurement update:
                for measure in measurements:
                    # Calculate current Landmark index (2*steps is first Landmark index, 2*measure[0] is Landmark Id)
                    landmark_index = 2 * self.steps + 2 * measure[0] + dim
                    # Distance is dx or dy in measurement to Landmark based on current dimension
                    distance = measure[dim + 1]
                    self._update_omega(index, landmark_index, self.measurement_noise)
                    self._update_xi(index, landmark_index, distance, self.measurement_noise)
                # Motion update:
                self._update_omega(index, index_2, self.motion_noise)
                self._update_xi(index, index_2, motion[dim], self.motion_noise)
        # Calculate mu vector with inverse omega matrix and xi vector
        omega_inverse = np.linalg.inv(np.matrix(self.omega))
        self.mu = omega_inverse * self.xi
        print('Done!')

    def get_poses_landmarks(self):
        """ Creates lists of Robot poses and landmarks positions from Mu vector """
        # Create list of poses
        poses = []
        for i in range(self.steps):
            poses.append((self.mu[2*i].item(), self.mu[2*i+1].item()))
        # Create list of landmarks
        landmarks = []
        for i in range(self.num_landmarks):
            landmarks.append((self.mu[2*(self.steps+i)].item(), self.mu[2*(self.steps+i)+1].item()))
        return poses, landmarks

    def show_results(self):
        """ Prints the resulting Robot poses and landmarks positions """
        poses, landmarks = self.get_poses_landmarks()
        print('\nSLAM results:')
        print('Estimated Positions:')
        for i in range(len(poses)):
            print('[' + ', '.join('%.3f' % p for p in poses[i]) + ']')
        print('\n')
        print('Estimated Landmarks:')
        for i in range(len(landmarks)):
            print('[' + ', '.join('%.3f' % land for land in landmarks[i]) + ']')

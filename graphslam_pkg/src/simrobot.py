#!/usr/bin/env python

import random

class Robot(object):
    """ Represents the robot with it's move and sense operations in 2D environment """
    def __init__(self, world_size, measurement_range, motion_noise, measurement_noise):
        self.world_size = world_size
        self.measurement_range = measurement_range
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        self.x = world_size / 2.0
        self.y = world_size / 2.0

    def __repr__(self):
        return f'Robot location: [x = {round(self.x, 3)}, y = {round(self.y, 3)}]'

    @staticmethod
    def random_num():
        """ Return random small number """
        return random.random() * 2.0 - 1.0

    def move(self, dx, dy):
        """ Performs the move operation with added randomness and motion noise,
            returns False if robot leaves the world borders """
        x = self.x + dx + self.random_num() * self.motion_noise
        y = self.y + dy + self.random_num() * self.motion_noise
        if x < 0.0 or x > self.world_size or y < 0.0 or y > self.world_size:
            return False
        else:
            self.x = x
            self.y = y
            return True

    def sense(self, landmarks):
        """ Performs sensing of landmarks in robot environment and calculates distances
            in x and y coordinates from robot to landmarks in measurement range.
            Mesurement shape is: [landmark_id, dx, dy] where dx and dy are distances from
            robot to that landmark """
        measurements = []
        for landmark_id, landmark in enumerate(landmarks):
            dx = landmark[0] - self.x + self.random_num() * self.measurement_noise
            dy = landmark[1] - self.y + self.random_num() * self.measurement_noise
            if abs(dx) <= self.measurement_range and abs(dy) <= self.measurement_range:
                measurements.append([landmark_id, dx, dy])
        return measurements

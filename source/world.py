from .robot import Robot
import random
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


class World(object):
    def __init__(self, world_size, num_landmarks):
        self.world_size = world_size
        self.num_landmarks = num_landmarks
        self.landmarks = []

    def make_landmarks(self):
        """ Makes landmarks at random locations in world based on defined number of landmarks """
        for i in range(self.num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size)])

    def make_data(self, steps, measurement_range, motion_noise, measurement_noise, distance):
        pass
#!/usr/bin/env python

from graphslam.robot import Robot
import random
import math
import matplotlib.pyplot as plt
import seaborn as sns

class World(object):
    def __init__(self, world_size, num_landmarks):
        self.world_size = world_size
        self.num_landmarks = num_landmarks
        self.distance = 20.0
        self.landmarks = None
        self.data = None

    def _make_landmarks(self):
        """ Makes landmarks at random locations in world based on defined number of landmarks """
        self.landmarks = []
        for i in range(self.num_landmarks):
            self.landmarks.append([round(random.random() * self.world_size),
                                   round(random.random() * self.world_size)])

    def _compute_dx_dy(self):
        """ Compute distances in x and y coordinates based on random rotation """
        orientation = random.random() * 2.0 * math.pi
        dx = math.cos(orientation) * self.distance
        dy = math.sin(orientation) * self.distance
        return dx, dy

    def make_data(self, steps, measurement_range, motion_noise, measurement_noise):
        """ Makes the data based on Robot repeated sensing and random movement inside world,
            stops when all landmarks are measured. Data is stored in shape [measurement, [dx, dy]],
            measurement is [landmark_id, x_distance, y_distance], dx and dy represents Robot movement
            at every step """
        print('Generating world...')
        robot = Robot(self.world_size, measurement_range, motion_noise, measurement_noise)
        self._make_landmarks()
        complete = False
        while not complete:
            self.data = []
            seen = [False for _ in range(self.num_landmarks)]
            dx, dy = self._compute_dx_dy()
            for step in range(steps-1):
                measurement = robot.sense(self.landmarks)
                for i in range(len(measurement)):
                    seen[measurement[i][0]] = True
                while not robot.move(dx, dy):
                    dx, dy = self._compute_dx_dy()
                self.data.append([measurement, [dx, dy]])
            complete = (sum(seen) == self.num_landmarks)
        print('\nTrue positions:\n')
        print(f'Landmarks: {self.landmarks}')
        print(robot)

    def display_world(self, robot_positions, landmark_positions):
        """ Plots the estimated Robot movement path and estimated landmark positions in world """
        sns.set_style('dark')
        ax = plt.gca()
        cols, rows = self.world_size + 1, self.world_size + 1
        ax.set_xticks([x for x in range(1, cols)], minor=True)
        ax.set_yticks([y for y in range(1, rows)], minor=True)
        # Plot grid on minor and major axes, major in larger width
        # plt.grid(which='minor', ls='-', lw=1, color='white')
        plt.grid(which='major', ls='-', lw=1.5, color='white')
        # Iterate over robot positions and plot the path and last location
        if len(robot_positions) > 1:
            for i in range(len(robot_positions) - 1):
                dx = robot_positions[i + 1][0] - robot_positions[i][0]
                dy = robot_positions[i + 1][1] - robot_positions[i][1]
                ax.arrow(robot_positions[i][0], robot_positions[i][1], dx, dy, head_width=1.5,
                         length_includes_head=True, color='gray')
            ax.text(robot_positions[-1][0], robot_positions[-1][1], 'o', ha='center', va='center',
                    color='r', fontsize=30)
        else:
            ax.text(robot_positions[-1][0], robot_positions[-1][1], 'o', ha='center', va='center',
                    color='r', fontsize=30)
        # Iterate over landmark positions and plot them on map
        for pos in landmark_positions:
            ax.text(pos[0], pos[1], 'x', ha='center', va='center', color='purple', fontsize=20)
        plt.rcParams["figure.figsize"] = (10, 10)
        plt.title('Robot and Landmark positions')
        plt.show()
        

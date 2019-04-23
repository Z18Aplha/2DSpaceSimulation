from Path import Path
from Point import Point
from math import sqrt, copysign
from Polynomial import Polynomial
from scipy.interpolate import interp1d
import numpy as np
from math import ceil
from PathPlanner import PathPlanner
import matplotlib.pyplot as plt


class Controller:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # each car has its own controller
    # class with path planning and path following algorithms

    def __init__(self, p: Path, max_acceleration, max_velocity, length):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.length_car = length        # length of the car, needed for steering angle (calculated with curvature)
        self.path = p               # list with points AND timestamps
        self.shape = []             # shape of path, the car should follow
        self.controls = []          # acceleration controls
        self.steer_control = []     # steering angle controls

    def calculate_controls(self, path): # currently: beziér curve degree 3
        planner = PathPlanner(path)
        self.shape = planner.generate_3()       # function generates shape (without timestamps)
        length = planner.get_section_length()   # length of each section (shape between two waypoints)
        curvature = planner.get_curvature()     # list of curvature values of the shape - needed for Ackerman steering
        steering = []
        for section in curvature:
            for curve in section:
                steering.append(np.arctan(self.length_car * curve)/np.pi * 180)
        plt.plot(steering)
        #plt.show()

        pass
        # fills the self.controls list with acceleration values
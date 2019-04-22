from Path import Path
from Point import Point
from math import sqrt, copysign
from Polynomial import Polynomial
from scipy.interpolate import interp1d
import numpy as np
from math import ceil
from PathPlanner import PathPlanner


class Controller:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # each car has its own controller
    # class with path planning and path following algorithms

    def __init__(self, p: Path, max_acceleration, max_velocity):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.path = p
        self.shape = []
        self.controls = []

    def calculate_controls(self, path): # currently: beziér curve degree 3
        planner = PathPlanner(path)
        self.shape = planner.generate_3()       # function generates shape (without timestamps)
        length = planner.get_section_length()

        # fills the self.controls list with acceleration values
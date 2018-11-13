import os
import pygame
from math import tan, radians, degrees, copysign, sqrt
import time


class PathFinder:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # exactly one instance of PathFinder needed

    def __init__(self, offset):
        self.safety_offset = offset  # offset of critical deceleration distance (now it's possible to stop the
        # vehicle on its destination

    def get_next(self, x_car, y_car, x_stop, y_stop, vx, vy, acceleration_max):
        # X-ACCELERATION
        if abs(x_stop - x_car) > 0.5 * vx * vx / acceleration_max[0] + self.safety_offset:
            ax = copysign(acceleration_max[0], (x_stop - x_car))  # copies sign of arg2 to value of arg1
        else:
            ax = -copysign(0.5 * vx * vx / (x_stop - x_car), vx)

        # Y-ACCELERATION
        if (abs(y_stop - y_car)) > 0.5 * vy * vy / acceleration_max[1] + self.safety_offset:
            ay = copysign(acceleration_max[1], (y_stop - y_car))  # copies sign of arg2 to value of arg1
        else:
            ay = -copysign(0.5 * vy * vy / (y_stop - y_car), vy)

        return [ax, ay]


class CarFree2D:
    def __init__(self, x: float, y: float, pathfinder: PathFinder):
        # PROPERTIES
        self.position = [x, y]  # in m - later: instantiation of 2d space with dates in metres
        self.length = 400  # [length] = cm
        self.width = 200  # [width] = cm
        self.timestamp = time.time()  # current time in UNIX
        self.pathfinder = pathfinder
        # VELOCITY (m/s)
        self.velocity = [0.0, 0.0]
        self.max_velocity = [50.0, 10.0]  # [vx, vy]
        # ACCELERATION (DE-) (m/s^2)
        self.acceleration = [0.0, 0.0]  # [ax, ay]
        self.max_acceleration = [30.0, 15.0]  # [ax, ay]
        # PATH
        self.destination = [x, y]  # coordinates of stop point (getting from 'God')

    # GETTER
    def get_position(self):
        return self.position

    def get_speed(self, absolute):  # absolute is boolean - true returns absolute value, false vectorial speed
        if absolute:
            return sqrt(self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1])
        else:
            return self.velocity

    def get_acceleration(self):
        return self.acceleration

    def status(self):
        return [self.position[0], self.position[1], self.destination[0], self.destination[1], self.velocity[0],
                self.velocity[1], self.acceleration[0], self.acceleration[1]]  # experimental, maybe further advantages?

    # SETTER
    def set_acceleration(self, ax: float, ay: float):
        # CHECK RANGE
        ax = max(-self.max_acceleration[0], min(ax, self.max_acceleration[0]))
        ay = max(-self.max_acceleration[1], min(ay, self.max_acceleration[1]))
        self.acceleration = [ax, ay]

    def set_velocity(self, vx: float, vy: float):
        # CHECK RANGE
        vx = max(-self.max_velocity[0], min(vx, self.max_velocity[0]))
        vy = max(-self.max_velocity[1], min(vy, self.max_velocity[1]))
        self.velocity = [vx, vy]

    def set_destination(self, x, y):
        self.destination = [x, y]

    # SIMULATION
    def update(self):

        dt = time.time() - self.timestamp  # [dt] = seconds
        self.timestamp = time.time()

        # ACCELERATION
        ax = self.pathfinder.get_next(self.position[0], self.position[1], self.destination[0], self.destination[1],
                                      self.velocity[0],
                                      self.velocity[1], self.max_acceleration)[0]
        ay = self.pathfinder.get_next(self.position[0], self.position[1], self.destination[0], self.destination[1],
                                      self.velocity[0],
                                      self.velocity[1], self.max_acceleration)[1]
        # idea: is there a decreasing or increasing process? Or instant values (small electric motor)?
        self.set_acceleration(ax, ay)

        # VELOCITY
        vx = self.velocity[0] + dt * self.acceleration[0]
        vy = self.velocity[1] + dt * self.acceleration[1]
        self.set_velocity(vx, vy)  # TODO getting clear about consistency of using setters and changing vars directly

        # POSITION
        x = self.position[0] + dt * self.velocity[0]
        y = self.position[1] + dt * self.velocity[1]
        self.position = [x, y]  # check TO-DO above!


class God:
    # TODO design omni-potent class
    # controls each car
    # interface to periphery
    # r/w from/to txt file (or sth else)
    # use (instance of CarFree2D).set_destination(x,y) to push data to each car
    # maybe: create a log file (CSV Data to copy it to excel etc to show graphs of velocity, acceleration, etc)...
    # ...for presentation
    pass


class SpaceSimulation2D:
    # TODO design Space for Simulation class
    # scalable - width/pixels = metres/Pixel
    # later: obstacles

    def __init__(self, x, y):
        self.high = y
        self.width = x
        self.px_high = 1920
        self.px_width = 1080
        self.high_ratio = self.px_high / self.high
        self.width_ratio = self.px_width / self.width
        # TODO complete instructor of SpaceSimulation2D


# TESTING ROUTINE


pf = PathFinder(5)
car1 = CarFree2D(0, 0, pf)
car1.set_destination(100, 100)
i = 0
while 1 < 2:
    i = i + 1
    car1.update()
    if (i % 100) == 0:  # skips showing some update routines - better overview
        print(car1.status())
print('test')

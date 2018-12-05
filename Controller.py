from Path import Path
from Point import Point
from math import sqrt, copysign
from Polynomial import Polynomial


class Controller:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # each car has its own controller
    # class with path planning and path following algorithms

    def __init__(self, p: Path, max_acceleration, max_velocity):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.path = p
        self.controls = []

    def check_possibility(self, last: Point, new: Point):  # recently: just for linear controlling
        distance = [abs(last.x - new.x), abs(last.y - new.y)]

        t_between = new.time - last.time

        distance_acceleration = [self.max_velocity[0] * self.max_velocity[0] / self.max_acceleration[0],
                                 self.max_velocity[1] * self.max_velocity[1] / self.max_acceleration[1]]
        if distance[0] > distance_acceleration[0] or distance[1] > distance_acceleration[1]:
            t_needed = max((2 * self.max_velocity[0] / self.max_acceleration[0] + (
                    distance[0] - distance_acceleration[0]) / self.max_velocity[0]), (
                                   2 * self.max_velocity[1] / self.max_acceleration[1] + (
                                   distance[1] - distance_acceleration[1]) / self.max_velocity[1]))
            possible = t_between > t_needed
        else:
            possible = True

        return possible

    def create_path(self, last: Point, destination: Point):
        # first step: linear

        t = destination.time - last.time
        distance = [abs(last.x - destination.x), abs(last.y - destination.y)]

        vx = -0.5 * (sqrt((-self.max_acceleration[0]) * (-self.max_acceleration[0] * t * t + 4 * distance[0])) -
                     self.max_acceleration[0] * t)
        vy = -0.5 * (sqrt((-self.max_acceleration[1]) * (-self.max_acceleration[1] * t * t + 4 * distance[1])) -
                     self.max_acceleration[1] * t)

        vx_max = max(-self.max_velocity[0], min(vx, self.max_velocity[0]))
        vy_max = max(-self.max_velocity[1], min(vy, self.max_velocity[1]))

        tx_to_max = vx_max / self.max_acceleration[0]
        ty_to_max = vy_max / self.max_acceleration[1]

        # STARTING POINT
        if tx_to_max > ty_to_max:
            # x component is limiting --> ax = maximum, ay needs to be adapted
            # ty_to_max must be equal to tx_to_max!
            t = tx_to_max
            ax = copysign(self.max_acceleration[0], destination.x - last.x)
            ay = copysign(distance[1] / distance[0] * ax, destination.y - last.y)
        else:
            # y component limits
            t = ty_to_max
            # ax = vx_max / t
            ay = copysign(self.max_acceleration[1], destination.y - last.y)
            ax = copysign(distance[0] / distance[1] * ay, destination.x - last.x)

        control = [Polynomial(0, 0, ax), Polynomial(0, 0, ay), last.time]
        if len(self.controls) > 0:
            if self.controls[-1] == control[2]:
                del self.controls[-1]
        self.controls.append(control)

        # ACCELERATION TO ZERO POINT
        control = [Polynomial(0, 0, 0), Polynomial(0, 0, 0), last.time + t]
        self.controls.append(control)

        # BRAKING POINT
        control = [Polynomial(0, 0, -ax), Polynomial(0, 0, -ay), destination.time - t]
        self.controls.append(control)

        # STOP POINT
        control = [Polynomial(0, 0, 0), Polynomial(0, 0, 0), destination.time]
        self.controls.append(control)

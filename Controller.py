from Path import Path
from Point import Point
from math import sqrt, copysign
from Polynomial import Polynomial
from scipy.interpolate import interp1d
import numpy as np
from math import ceil


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

    def create_path_linear_event(self, last: Point, destination: Point):
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

    def create_path_spline_equidistant(self, path, c_dt):
        # CREATING SPLINES FOR EACH DIRECTION
        x = []
        y = []
        t = []

        for point in path:
            x.append(point.x)
            y.append(point.y)
            t.append(point.time)

        spline_x = interp1d(t, x, kind="cubic")
        spline_y = interp1d(t, y, kind="cubic")

        # CALCULATING ACCELERATION VALUES
        n = ceil(t[-1]*1000/c_dt)   # #inputs of the controller
        t_new = np.linspace(0, t[-1], num=n, endpoint=True)     # each time of controller input
        x_new = spline_x(t_new)     # spline at each time
        y_new = spline_y(t_new)

        self.path.points = []
        for i in range(0, n-1):
            self.path.add(Point(x_new[i], y_new[i], t_new[i], True))

        vx = []
        vy = []
        vx.append(0)
        vy.append(0)
        for i in range(1, n-1):
            vx.append((x_new[i] - x_new[i-1])/(c_dt/1000))
            vy.append((y_new[i] - y_new[i-1])/(c_dt/1000))

        for i in range(1, n-1):
            ax = ((vx[i] - vx[i-1])/(c_dt/1000))
            ay = ((vy[i] - vy[i-1])/(c_dt/1000))
            control = [Polynomial(0, 0, ax), Polynomial(0, 0, ay), c_dt*(i-1)/1000]
            self.controls.append(control)
        pass



            # Liste mit "Ableitungen" zwischen jedem Punkte von x/y_new --> Geschwindigkeit
            # aus dieser Liste soll jetzt Liste für Beschleunigungen werden
            # Frage: einfach liste machen und werte aller c_dt entnehmen oder andere algorithmus zur bestimmung?
            # bei sehr großen n --> einfach entsprechenden wert

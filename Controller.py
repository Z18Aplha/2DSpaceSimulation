from Path import Path
from Point import Point
from math import sqrt, copysign
from Polynomial import Polynomial
from scipy.interpolate import interp1d
import numpy as np
from math import ceil
import matplotlib.pyplot as plt


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

        #hermx = np.polynomial.hermite.hermfit(t, x, 3)
        #hermy = np.polynomial.hermite.hermfit(t, y, 3)

        #print(hermx)
        #print(hermy)

        # CALCULATING ACCELERATION VALUES
        n = ceil(t[-1] * 1000 / c_dt)  # #inputs of the controller
        t_new = np.linspace(0, t[-1], num=n, endpoint=True)  # each time of controller input
        x_new = spline_x(t_new)  # spline at each time
        y_new = spline_y(t_new)

        #plt.plot(t_new, x_new)
        #plt.plot(t_new, y_new)
        #plt.show()

        self.path.points = []
        for i in range(0, n):
            self.path.add(Point(x_new[i], y_new[i], t_new[i], True))

        # CALCULATING DERIVATIVES OF SPLINE WITH c_dt --> (y(x2)-y(x1))/(x2-x1) --> Definition of derivative
        vx = []
        vy = []
        vx.append(0)
        vy.append(0)
        for i in range(1, n):
            vx.append((x_new[i] - x_new[i - 1]) / (c_dt / 1000))
            vy.append((y_new[i] - y_new[i - 1]) / (c_dt / 1000))

        for i in range(1, n):
            ax = ((vx[i] - vx[i - 1]) / (c_dt / 1000))
            ay = ((vy[i] - vy[i - 1]) / (c_dt / 1000))
            control = [Polynomial(0, 0, ax), Polynomial(0, 0, ay), c_dt * (i - 1) / 1000]
            self.controls.append(control)
        pass

        # RECALCULATING LAST DECELERATION VALUE TO BE SURE THAT THE CAR STOPS
        ax = -vx[-2]/(c_dt/1000)
        ay = -vy[-2]/(c_dt/1000)
        control = [Polynomial(0, 0, ax), Polynomial(0, 0, ay), t[-10]-c_dt/1000]
        self.controls[-1] = control

        # SET DECELERATION TO "true" ZERO
        control = [Polynomial(0, 0, 0), Polynomial(0, 0, 0), t[-10]]
        self.controls.append(control)

    def spline_other_method(self, path, c_dt):
        x = []
        y = []
        t = []

        for point in path:      # converting Path into 1D-lists for the spline interpolation
            x.append(point.x)
            y.append(point.y)
            t.append(point.time)

        spline_x = interp1d(t, x, kind="cubic")
        spline_y = interp1d(t, y, kind="cubic")

        # CALCULATING LENGTH OF EACH SECTION OF SPLINE (and sum)
        spline_length_sections = []

        ratio = 100    # number of equidistant calculation points in 1s --> 50 means 50 points every one second

        for i in range(1, len(t)):
            time_start = t[i-1]     # start and stop of section of spline between two points
            time_stop = t[i]
            time = time_start
            c = 0
            while time <= time_stop-1/ratio:
                c += (sqrt((spline_x(time)-spline_x(time+1/ratio))**2+(spline_y(time)-spline_y(time+1/ratio))**2))
                time += 1/ratio
            spline_length_sections.append(c)

        spline_length = 0
        for length in spline_length_sections:
            spline_length += length
        print(spline_length_sections)
        print(spline_length)

        # CALCULATE TIME OF ACCELERATION (absolute, not for each component)
        v0 = 0  # velocity of previous sector
        t_new = []
        ax = []
        ay = []
        vx = []
        vy = []
        sx = []
        sy = []
        for i in range(0, len(spline_length_sections)):
            duration = t[i+1] - t[i]                # duration of this sector
            distance = spline_length_sections[i]    # distance to drive in this time
            a_max = sqrt(self.max_acceleration[0]**2+self.max_acceleration[1]**2)      # abs value of acceleration
            t_acceleration = 0                      # time of a_max
            t_deceleration = 0                      # time of -a_max

            # TIME OF MAX ACCELERATION AND DECELERATION
            if self.path.points[i+1].is_destination:  # TRUE for Destination
                v1 = sqrt(-4*a_max*distance+(a_max*duration)**2+2*a_max*duration*v0-v0**2)+a_max*duration+v0
                v1 = v1/2
                t_deceleration = v1/a_max
                t_acceleration = (v1-v0)/a_max
            else:   # FALSE for Waypoint
                v1 = -sqrt((a_max*duration)**2 - 2*a_max*distance+v0**2)+a_max*duration
                t_acceleration = (v1-v0)/a_max

            # now: knowing time of acceleration and deceleration with a_max

            # CALCULATION OF ACCELERATION COMPONENTS (small grid)
            t_grid = t[i]
            while t_grid + 1/ratio <= t[i+1]:
                if t_grid - t[i] < t_acceleration:
                    dx = spline_x(t_grid+1/ratio) - spline_x(t_grid)
                    dy = spline_y(t_grid+1/ratio) - spline_y(t_grid)
                    a_y = -dx/(2*dy) + sqrt((dx/(2*dy))**2 + a_max**2)
                    a_x = (dx/dy) * a_y
                    ax.append(a_x)
                    ay.append(a_y)
                elif t_grid - t[i+1] + 1/ratio >= t_deceleration:
                    ax.append(0)
                    ay.append(0)
                else:
                    dx = spline_x(t_grid + 1 / ratio) - spline_x(t_grid)
                    dy = spline_y(t_grid + 1 / ratio) - spline_y(t_grid)
                    a_y = -dx / (2 * dy) + sqrt((dx / (2 * dy)) ** 2 + a_max ** 2)
                    a_x = (dx / dy) * a_y
                    ax.append(-a_x)
                    ay.append(-a_y)
                t_new.append(t_grid)
                t_grid += 1/ratio

        plt.plot(t_new, ax)
        plt.plot(t_new, ay)
        plt.show()


        # well.. i stopped here, because i found a (bad) way to force the boundary conditions
        # nevertheless, the hermite conditions (f'(start) = f'(stop) = 0) are much better and reduces code
        # we just need to find a better way to implement them --> but for the moment: it works (thumbs up)

        # i could delete this funtion/method (however in Python), but maybe, we need some pieces of it in further code


        # the next stuff are TODOs and mind support for myself (because this stuff needed more than one session :D)

        # ähnlich wie bei lin muss jetzt abs. Beschleunigung bestimmt werden
        # Fahrzeug wird t lang beschleunigt, bis entrsprechendes Tempo erreicht
        # absolute Beschleunigungswerte werden auf Spline übertragen, indem der Winkel aus Punkt-Paaren gebildet
        # und die ax und ay Werte errechnet werden
        # somit sollte es möglich sein, einen Wegpunkt und Zielpunkt zu realisieren

        # Schleife muss Geschwindigkeit bei Durchfahrt kennen
        # vmax und amax dürfen nicht überschritten werden --> sollten bekannt sein
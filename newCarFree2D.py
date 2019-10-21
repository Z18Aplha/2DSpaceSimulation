from Path import Path
from math import sqrt, cos, sin, atan
from Point import Point
from Polynomial import Polynomial
from newPathPlanner import PathPlanner
from Controller import Controller
import numpy as np
from Event import Event
import math as m
import Lib as lib


class CarFree2D:
    def __init__(self, id: int, spawn_x, spawn_y, size_x, size_y, angle, max_vel, max_acc, color, ts):
        self.ghost = False
        # PHYSICAL PROPERTIES
        self.color = color                          # color code
        self.id = id                                # unique car id
        self.spawn = [spawn_x, spawn_y]             # spawnpoint
        self.position_x = []                          # not used for EventQueue
        self.position_y = []
        self.direction = angle                      # direction of the car
        self.start_direction = angle                # start-direction of the car
        self.last_position = [spawn_x, spawn_y]     # position after last control input
        self.length = size_x                        # in m
        self.width = size_y                         # in m
        # VELOCITY
        self.last_velocity = 0
        self.last_velocity_x = 0                      # velocity after last control input
        self.last_velocity_y = 0
        self.velocity = []                          # not used for EventQueue
        self.max_velocity = max_vel                 # absolute limit of velocity
        # ACCELERATION
        self.acceleration = 0                       # acceleration given by last control input
        self.acceleration_y = 0
        self.acceleration_x = 0
        self.max_acceleration = max_acc             # absolute limit of acceleration
        # STEERING
        self.steering = 0                           # steering angle given by last control input
        self.direction = 0                          # direction angle given by last control input
        # PATH
        self.shape = []                             # shape of the planned path (without exact timestamp)
        self.path = Path(self.spawn)                # "shape" with exact timestamp
        self.waypoints = []                         # list for the given points with the
        self.ts = ts/1000                           # sampling time of AGV^(-1)
        self.t_to_length = []                       # t is parameter of bezier-curve, t_to_length is correlation of t and length of shape
        self.planner = None
        # CONTROLS
        self.time_last_control = 0                  # timestamp of last control input
        self.time_last_update = 0                   # timestamp of last given acceleration
        self.stop = False                           # False: car drives, True: car stops (or will stop within the next time (t < ts)
        self.stop_time = 0                          # timestamp of stop (velocitiy 0 reached)
        self.control_prep = []
        self.controller = Controller(self, lib.k_p, lib.k_d)
        # DEBUGGING
        self.debugging1 = []
        self.debugging2 = []
        self.dc_pos = []
        dim = len(lib.statespace.A)
        self.old_state = [np.zeros(dim).reshape(-1, 1), np.zeros(dim).reshape(-1, 1)]
        self.state = [np.zeros(dim), np.zeros(dim)]
        self.counter = 0

    # GETTER
    # currenttly not used
    def get_speed(self, absolute):  # absolute is boolean - true returns absolute value, false vectorial speed
        if absolute:
            return sqrt(self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1])
        else:
            return self.velocity

    # currenttly not used
    def get_acceleration(self):
        return self.acceleration

    # sets waypoint (given by *.json file) for the car
    def set_waypoint(self, x, y):
        p = Point(x, y)
        self.path.add(p)
        self.waypoints.append(p)
        # raise Exception('The point (' + str(p.x) + '|' + str(p.y) + ') is too far away. Skipped.')

    # creates spline for the given waypoints (from the *.json file)
    def create_spline(self):
        self.planner = PathPlanner(self.max_velocity, self.max_acceleration)
        self.planner.make_path(self.path.points)
        self.write_path()
        if not self.ghost:
            self.make_controls()
        self.stop_time = self.planner.t_equi_in_t[-1]

    def write_path(self):
        for point in self.planner.path_from_v_equi_in_t:
            self.shape.append([np.real(point), np.imag(point)])
        pass

    def test_dc_motor(self, ax, ay):
        self.counter += 1
        # x direction:
        test = lib.statespace.B.A.dot(ax)
        test2 = lib.statespace.A.A.dot(self.old_state[0])
        self.state[0] = lib.statespace.A.A.dot(self.old_state[0]) + lib.statespace.B.A.dot(ax)
        x = lib.statespace.C.A.dot(self.old_state[0]) + lib.statespace.D.A.dot(ax)
        # y direction:
        self.state[1] = lib.statespace.A.A.dot(self.old_state[1]) + lib.statespace.B.A.dot(ay)
        y = lib.statespace.C.A.dot(self.old_state[1]) + lib.statespace.D.A.dot(ay)

        self.old_state = self.state
        return x[0][0], y[0][0]

    # used with EventQueue
    # car gets controlled with specific values by an Event (car_control)
    def steer(self, t, acc_x, acc_y, stop):

        dt = t - self.time_last_control
        #dt = self.ts

        x = (0.5*(dt**2) * self.acceleration_x) + self.last_velocity_x * dt + self.last_position[0]
        y = (0.5*(dt**2) * self.acceleration_y) + self.last_velocity_y * dt + self.last_position[1]

        self.last_position = [x, y]

        self.last_velocity_x += self.acceleration_x * dt
        self.last_velocity_y += self.acceleration_y * dt
        self.last_velocity = sqrt(self.last_velocity_x**2 + self.last_velocity_y**2)

        # update (steer) the car
        self.time_last_control = t
        self.acceleration = sqrt(acc_x**2 + acc_y**2)
        self.stop = stop
        self.position_x.append(x)
        self.position_y.append(y)
        # save new acceleration
        self.acceleration_x = acc_x
        self.acceleration_y = acc_y
        self.debugging1.append([t, self.acceleration])
        if stop:
            # self.stop_time = self.last_velocity / self.acceleration + self.time_last_control
            self.stop_time = t
        else:
            comp_vel = complex(self.last_velocity_x, self.last_velocity_y)
            self.direction = np.angle([comp_vel])[0]

    def control(self, t, a_x, a_y):
        self.acceleration_x += a_x
        self.acceleration_y += a_y
        dt = t - self.time_last_control
        # dt = self.ts

        x = (0.5 * (dt ** 2) * self.acceleration_x) + self.last_velocity_x * dt + self.last_position[0]
        y = (0.5 * (dt ** 2) * self.acceleration_y) + self.last_velocity_y * dt + self.last_position[1]

        self.last_position = [x, y]

        self.last_velocity_x += self.acceleration_x * dt
        self.last_velocity_y += self.acceleration_y * dt
        self.last_velocity = sqrt(self.last_velocity_x ** 2 + self.last_velocity_y ** 2)

        # update (control) the car
        self.time_last_control = t
        self.acceleration = sqrt(self.acceleration_x ** 2 + self.acceleration_y ** 2)
        self.debugging1.append([t, self.acceleration])
        comp_vel = complex(self.last_velocity_x, self.last_velocity_y)
        self.direction = np.angle([comp_vel])[0]

    # CONVERTING CONTROL_PREP INTO CONTROLS FOR CAR
    # [timestamp, estimated x, estimated y, abs(acceleration), direction to drive]
    def make_controls(self):
        stop_time = self.planner.t_equi_in_t[-1]
        stop = False
        t = 0
        for acc in self.planner.acceleration_from_v_equi_in_t:
            if acc == self.planner.acceleration_from_v_equi_in_t[-1]:
                stop = True
            ev = Event(t, self, (t, self, np.real(acc), np.imag(acc), stop, "steering"), lambda: lib.eventqueue.car_steering)
            lib.eventqueue.add_event(ev)
            t += lib.dt
        # fills the self.controls list with acceleration values
        self.controller.set_path(self.planner.path_from_v_equi_in_t)

    # used with EventQueue
    # returns position of the car at specific time t
    # also more information available (but certainly not needed),f.e. acceleration, direction, steering, angle, ...
    def get_data(self, t):
        if self.ghost:
            try:
                index = int(t / lib.dt)
                point = self.planner.path_from_v_equi_in_t[index]
                vel = self.planner.velocity_from_v_equi_in_t[index]
                dir = np.angle([vel])
            except IndexError:
                point = self.planner.path_from_v_equi_in_t[-1]
                vel = self.planner.velocity_from_v_equi_in_t[-1]
                vel_dir = self.planner.velocity_from_v_equi_in_t[-2]
                dir = np.angle([vel_dir])

            vel = abs(vel)
            x = point.real
            y = point.imag

        else:
            if not self.stop:
                dt = (t - self.time_last_control)

            else:
                dt = (self.stop_time - self.time_last_control)

            x = 0.5 * (dt ** 2) * m.cos(self.direction) * self.acceleration + self.last_velocity_x * dt + self.last_position[0]
            y = 0.5 * (dt ** 2) * m.sin(self.direction) * self.acceleration + self.last_velocity_y * dt + self.last_position[1]

            # x = self.last_position[0]
            # y = self.last_position[1]

            dir = round(self.direction, 5)

            vel = self.last_velocity

        if t == 0:
            dir = self.start_direction
        return [t, self.id, round(x, 4), round(y, 4), vel, dir]


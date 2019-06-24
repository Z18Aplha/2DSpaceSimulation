from Path import Path
from Point import Point
from math import sqrt, copysign
from Polynomial import Polynomial
from scipy.interpolate import interp1d
import numpy as np
from math import ceil
from PathPlanner import PathPlanner
import matplotlib.pyplot as plt
import math as m
from Event import Event

def angle(p1: Point, p2: Point):
    phi = m.atan2(p2.y - p1.y, p2.x - p1.x)
    return phi


class Controller:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # each car has ic_dt own controller
    # class with path planning and path following algorithms


    def __init__(self, p: Path, max_acceleration, max_velocity, length):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.length_car = length  # length of the car, needed for steering angle (calculated with curvature)
        self.path = p  # list with poinc_dt AND timestamps
        self.shape = []  # shape of path, the car should follow
        self.controls = []  # acceleration controls
        self.steer_control = []  # steering angle controls
        self.t_to_length = []  # t-to-length connection

    def calculate_controls_equidistant(self, path, c_dt):  # currently: beziér curve degree 3
        # CREATING NECESSARY VARIABLES
        planner = PathPlanner(path)
        self.shape = planner.generate_3()  # function generates shape (without timestamps)
        length = planner.get_section_length()  # length of each section (shape between two waypoinc_dt)
        length_abs = 0
        for section in length:
            length_abs += section
        self.t_to_length = planner.t_to_length
        curvature = planner.get_curvature()  # list of curvature values of the shape - needed for Ackerman steering
        steering = []  # steering angle of car
        for section in curvature:
            for curve in section:
                steering.append(np.arctan(self.length_car * curve) / np.pi * 180)
        # plt.plot(steering)
        # plt.show()

        # DEFINE NUMBER OF SAMPLES WITH MAX ACCELERATION
        t_ac = self.max_velocity / self.max_acceleration  # time to max velocity
        s_ac = Polynomial(0, -self.max_acceleration, self.max_velocity).integration().get_value(
            t_ac)  # way of deceleration
        # t_ac = np.ndarray.tolist(t_ac)
        c_dt = c_dt

        if t_ac % c_dt == 0:
            adaption_needed = False
        else:
            adaption_needed = True

        n_ac_max = int(t_ac / c_dt) - 1  # number of samples with maximum acceleration

        # PREPARE CONTROLS: FILL LIST WITH ACC_MAX, CALCULATE DISTANCE TRAVELLED
        # control_prep: [timestamp, acceleration, distance traveled, velocity]
        # TODO works not for (to) short shapes --> need to be added!!!
        control_prep = []
        distance = 0
        t = 0
        a = self.max_acceleration
        i = 0
        for k in range(0, n_ac_max):
            t = round(k * c_dt, 7)
            #distance = a.integration().integration().get_value(t)
            distance = 0.5*a*t**2
            vel = a*t
            control_prep.append([t, a, round(distance, 7), vel])
            i = k

        #distance = a.integration().integration().get_value(t + c_dt)
        distance = 0.5*a*t**2
        vel = a*(t + c_dt)

        if adaption_needed:
            a_value = ((t_ac - c_dt * n_ac_max) * self.max_acceleration) / (2 * c_dt)
            a = a_value
            distance_ref = distance
            vel_ref = vel
            for j in range(n_ac_max, n_ac_max + 2):
                t = round((j - i) * c_dt, 7)
                control_prep.append([round(j * c_dt, 7), a, round(distance, 7), vel])
                distance = distance_ref + 0.5 * a_value * t ** 2 + vel_ref * t
                vel += a_value * c_dt

        # ADD CONTROLS WITH 0 ACCELERATION TO THE LIST
        # OFFSET NEEDED TO ADAPT THE DEC VALUES TO THE SAMPLING RATIO
        # if adaption_needed:
        #    offset = distance
        # else:
        #    offset = 0

        distance_ref = distance
        distance_diff = vel * c_dt
        while distance < length_abs - distance_ref - distance_diff:
            j += 1
            control_prep.append([round(j * c_dt, 7), 0, round(distance, 7), vel])
            distance += distance_diff

        # ADD DEC VALUES TO THE LIST
        a_value = abs(vel ** 2 / (2 * (length_abs - distance)))
        n = j
        vel_ref = vel
        distance_ref = distance
        while vel >= 0:
            j += 1
            t = round(((j - n) * c_dt), 7)
            control_prep.append([round(j * c_dt, 7), -a_value, round(distance, 7), vel])
            vel -= a_value * c_dt
            distance = distance_ref + -a_value * 0.5 * t ** 2 + vel_ref * t

        vel = control_prep[-1][3]
        t = control_prep[-1][0]
        s = control_prep[-1][2]
        t_stop = t + vel/a_value
        s_stop = s + -a_value*0.5*(vel/a_value)**2 + vel*(vel/a_value)
        control_prep.append([t_stop, s_stop, 'CAR STOPPED'])

        # CONTROLLER GEc_dt COORDINATES FOR EACH ENTRY IN CONTROL PREP
        for i in range(0, len(control_prep)-1):
            xy = planner.get_coordinates(control_prep[i][2])
            control_prep[i] = [control_prep[i][0], control_prep[i][1], control_prep[i][3], xy[0], xy[1]]

        # CONVERTING CONTROL_PREP INTO CONTROLS FOR CAR
        # [timestamp, estimated x, estimated y, abs(acceleration), direction to drive]
        stop = False
        for control in control_prep:
            if control[2] == 'CAR STOPPED':
                pass
            else:
                t = control[0]
                est_x = control[3]
                est_y = control[4]
                est_point = Point(est_x, est_y)
                acc = control[1]
                try:
                    next_x = control_prep[control_prep.index(control)+1][3]
                    next_y = control_prep[control_prep.index(control)+1][4]
                    next_point = Point(next_x, next_y)
                    dir = angle(est_point, next_point)
                except IndexError:
                    stop = True

            self.controls.append([t, est_x, est_y, acc, dir, stop])
#            ev = Event(t, )

        self.controls.pop(-1)

        pass
        # fills the self.controls list with acceleration values

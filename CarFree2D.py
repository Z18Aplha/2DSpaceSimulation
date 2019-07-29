from Controller import Controller
from Path import Path
from math import sqrt, cos, sin
from Point import Point
from Polynomial import Polynomial
from PathPlanner import PathPlanner
import numpy as np
from Event import Event
import math as m
import Lib as lib


class CarFree2D:
    def __init__(self, id: int, spawn_x, spawn_y, size_x, size_y, angle, max_vel, max_acc, color, ts):
        # PHYSICAL PROPERTIES
        self.color = color                          # color code
        self.id = id                                # unique car id
        self.spawn = [spawn_x, spawn_y]             # spawnpoint
        self.position = []                          # not used for EventQueue
        self.direction = angle                      # direction of the car
        self.last_position = [spawn_x, spawn_y]     # position after last control input
        self.length = size_x                        # in m
        self.width = size_y                         # in m
        # VELOCITY
        self.last_velocity = 0                      # velocity after last control input
        self.velocity = []                          # not used for EventQueue
        self.max_velocity = max_vel                 # absolute limit of velocity
        # ACCELERATION
        self.acceleration = 0                       # acceleration given by last control input
        self.max_acceleration = max_acc             # absolute limit of acceleration
        # STEERING
        self.steering = 0                           # steering angle given by last control input
        self.direction = 0                          # direction angle given by last control input
        # PATH
        self.shape = []                             # shape of the planned path (without exact timestamp)
        self.path = Path(self.spawn)                # "shape" with exact timestamp
        self.waypoints = []                         # list for the given points with the
        self.controller = Controller(self.path, self.max_acceleration, self.max_velocity, self.length)
        self.ts = ts/1000                           # sampling time of AGV^(-1)
        self.t_to_length = []                       # t is parameter of bezier-curve, t_to_length is correlation of t and length of shape
        # CONTROLS
        self.controls = []                          # not used for EventQueue
        self.time_last_control = 0                  # timestamp of last control input
        self.stop = False                           # False: car drives, True: car stops (or will stop within the next time (t < ts)
        self.stop_time = 0                          # timestamp of stop (velocitiy 0 reached)

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

    # not usable with EventQueue
    # returns cars status for a given time t
    def status(self, t):

        # INITIALIZATION
        ax = 0
        ay = 0
        vx = 0
        vy = 0
        sx = self.spawn[0]
        sy = self.spawn[1]

        # ACCELERATION
        t_old = 0
        for control in self.controller.controls:
            if control[2] <= t:
                t_old = control[2]
                dt = t - t_old
                ax = control[0].get_value(dt)
                ay = control[1].get_value(dt)

        # VELOCITY
        t_old = 0
        for velocity in self.velocity:
            if velocity[2] <= t:
                t_old = velocity[2]
                dt = t - t_old
                vx = velocity[0].get_value(dt)
                vy = velocity[1].get_value(dt)

        # POSITION
        t_old = 0
        for position in self.position:
            if position[2] <= t:
                t_old = position[2]
                dt = t - t_old
                sx = position[0].get_value(dt)
                sy = position[1].get_value(dt)

        return [self.id, t, sx, sy, vx, vy, ax, ay]

    # sets waypoint (given by *.json file) for the car
    def set_waypoint(self, x, y):
        p = Point(x, y)
        self.path.add(p)
        self.waypoints.append(p)
        # raise Exception('The point (' + str(p.x) + '|' + str(p.y) + ') is too far away. Skipped.')

    # not usable with EventQueue
    # calculates the behaviour of the car for the whole time (only one time)
    # works with x and y components
    def update(self):   # TODO refactor this entry --> acc and vel no vector anymore

        # VELOCITY
        vx = Polynomial(0, 0, 0)
        vy = Polynomial(0, 0, 0)
        t_old = 0
        velocity_x_old = 0
        velocity_y_old = 0
        for control in self.controller.controls:
            # Control: [timestamp, ax, ay]
            dt = control[0] - t_old
            velocity_x_old = vx.get_value(dt)
            velocity_y_old = vy.get_value(dt)
            vx = control[1].integration().add_constant(velocity_x_old)
            vy = control[2].integration().add_constant(velocity_y_old)
            t_old = control[0]
            self.velocity.append([vx, vy, control[0]])

        # POSITION
        sx = Polynomial(0, 0, self.spawn[0])
        sy = Polynomial(0, 0, self.spawn[1])
        t_old = 0
        for entry in self.velocity:
            dt = entry[2] - t_old
            sx_old = sx.get_value(dt)
            sy_old = sy.get_value(dt)
            sx = entry[0].integration().add_constant(sx_old)
            sy = entry[1].integration().add_constant(sy_old)
            t_old = entry[2]
            self.position.append([sx, sy, entry[2]])

    # not usable with EventQueue
    # calculates the behaviour of the car for the whole time (only one time)
    # works with absolut values and angles
    def update2(self):
        # new Update routine

        # POSITION and VELOCITY
        x = self.spawn[0]
        y = self.spawn[1]
        t = 0
        v = 0
        for control in self.controls:
            # Control: [timestamp, est_x, est_y, a, dir, stop]
            stop = control[5]
            t = round(control[0], 7)
            direction = control[4]
            a = control[3]
            self.velocity.append([t, v, direction])  # [t, v, dir]
            self.position.append([t, x, y])
            if stop:
                t_stop = t - v / a
                dt = t-t_stop
                x += (0.5*a*dt**2 + v*dt)*cos(direction)
                y += (0.5*a*dt**2 + v*dt)*sin(direction)
                t = t_stop
                a = 0
                v = 0
                self.velocity.append([t, v, direction])  # [t, v, dir]
                self.position.append([t, x, y])
                break
            x += (0.5*a*self.ts**2 + v*self.ts) * cos(direction)
            y += (0.5*a*self.ts**2 + v*self.ts) * sin(direction)
            v += a * self.ts

        pass

    # creates spline for the given waypoints (from the *.json file)
    def create_spline(self):
        self.calculate_controls_equidistant(self.path.points)

    # not usable with EventQueue
    # calculates the controls for the car for the whole time (only one time)
    # works with absolute values and angles
    def calculate_controls_equidistant(self, path):  # currently: beziér curve degree 3
        print('Car', self.id)
        # CREATING NECESSARY VARIABLES
        planner = PathPlanner(path)
        self.shape = planner.generate_3()  # function generates shape (without timestamps)
        length = planner.get_section_length()  # length of each section (shape between two waypoints)
        length_abs = 0
        for section in length:
            length_abs += section
        self.t_to_length = planner.t_to_length
        curvature = planner.get_curvature()  # list of curvature values of the shape - needed for Ackerman steering
        steering = []  # steering angle of car
        for section in curvature:
            for curve in section:
                steering.append(np.arctan(self.length * curve) / np.pi * 180)
        # plt.plot(steering)
        # plt.show()

        # DEFINE NUMBER OF SAMPLES WITH MAX ACCELERATION
        t_ac = self.max_velocity / self.max_acceleration  # time to max velocity
        s_ac = Polynomial(0, -self.max_acceleration, self.max_velocity).integration().get_value(
            t_ac)  # way of deceleration
        # t_ac = np.ndarray.tolist(t_ac)

        if t_ac % self.ts == 0:
            adaption_needed = False
        else:
            adaption_needed = True

        n_ac_max = int(t_ac / self.ts) - 1  # number of samples with maximum acceleration

        # PREPARE CONTROLS: FILL LIST WITH ACC_MAX, CALCULATE DISTANCE TRAVELLED
        # control_prep: [timestamp, acceleration, distance traveled, velocity]
        # TODO works not for (to) short shapes --> need to be added!!!
        control_prep = []
        distance = 0
        t = 0
        a = self.max_acceleration
        i = 0
        for k in range(0, n_ac_max):
            t = round(k * self.ts, 7)
            # distance = a.integration().integration().get_value(t)
            distance = 0.5 * a * t ** 2
            vel = a * t
            control_prep.append([t, a, round(distance, 7), vel])
            i = k

        # distance = a.integration().integration().get_value(t + ts)
        distance = 0.5 * a * t ** 2
        vel = a * (t + self.ts)

        if adaption_needed:
            a_value = ((t_ac - self.ts * n_ac_max) * self.max_acceleration) / (2 * self.ts)
            a = a_value
            distance_ref = distance
            vel_ref = vel
            for j in range(n_ac_max, n_ac_max + 2):
                t = round((j - i) * self.ts, 7)
                control_prep.append([round(j * self.ts, 7), a, round(distance, 7), vel])
                distance = distance_ref + 0.5 * a_value * t ** 2 + vel_ref * t
                vel += a_value * self.ts

        # ADD CONTROLS WITH 0 ACCELERATION TO THE LIST
        # OFFSET NEEDED TO ADAPT THE DEC VALUES TO THE SAMPLING RATIO
        # if adaption_needed:
        #    offset = distance
        # else:
        #    offset = 0

        distance_ref = distance
        distance_diff = vel * self.ts
        while distance < length_abs - distance_ref - distance_diff:
            j += 1
            control_prep.append([round(j * self.ts, 7), 0, round(distance, 7), vel])
            distance += distance_diff

        # ADD DEC VALUES TO THE LIST
        a_value = abs(vel ** 2 / (2 * (length_abs - distance)))
        n = j
        vel_ref = vel
        distance_ref = distance
        while vel >= 0:
            j += 1
            t = round(((j - n) * self.ts), 7)
            control_prep.append([round(j * self.ts, 7), -a_value, round(distance, 7), vel])
            vel -= a_value * self.ts
            distance = distance_ref + -a_value * 0.5 * t ** 2 + vel_ref * t

        vel = control_prep[-1][3]
        t = control_prep[-1][0]
        s = control_prep[-1][2]
        t_stop = t + vel / a_value
        s_stop = s + -a_value * 0.5 * (vel / a_value) ** 2 + vel * (vel / a_value)
        control_prep.append([t_stop, s_stop, 'CAR STOPPED'])

        # CONTROLLER GEts COORDINATES FOR EACH ENTRY IN CONTROL PREP
        # diese Schleife verzögert das Programm bei einem langen Weg eines Autos enorm (bei 1ms Sampling)
        # Lösung könnte eine Übergabe einer gesamten Liste sein, bei jedem Aufruf wird wieder eine Schleife gestartet (in der get_coordinates)
        # Wenn also die die control_prep im ganzen Übergeben und bearbeitet würde, könnte ein bisschen Zeit gespart werden.
        # vielleicht sogar genug, damit es nicht mehr (so sehr)
        # vorher wurde bereits eine Liste mit [Zeitpunkt, Beschleunigungswert, ingesemat zurückgelegte Strecke] erstellt
        # diese Funktion berechnet jetzt die entsprechenden Koordinaten auf der Bezier-Curve (Stichpunkt Parameter der Kurve)
        # Die Schleife ruft jeden Punkt in der vorher erstellen Liste einzeln auf (das könnte das Problem sein)
        # xy ist einfach eine Liste der Form [x, y]
        for i in range(0, len(control_prep) - 1):
            xy = planner.get_coordinates(control_prep[i][2]) # siehe PathPlanner
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
                    next_x = control_prep[control_prep.index(control) + 1][3]
                    next_y = control_prep[control_prep.index(control) + 1][4]
                    next_point = Point(next_x, next_y)
                    dir = lib.angle(est_point, next_point)
                except IndexError:
                    stop = True

            self.controls.append([t, est_x, est_y, acc, dir, stop])
            ev = Event(t, self, (t, self, acc, dir, stop), lambda: lib.eventqueue.car_control)
            lib.eventqueue.add_event(ev)

        self.controls.pop(-1)

        pass
        # fills the self.controls list with acceleration values

    # used with EventQueue
    # car gets controlled with specific values by an Event (car_control)
    def control(self, t, acc, direction, stop):
        # save current position and speed
        dt = t - self.time_last_control
        x = 0.5*(dt**2) * m.cos(direction) * acc + m.cos(direction) * self.last_velocity * dt + self.last_position[0]
        y = 0.5*(dt**2) * m.sin(direction) * acc + m.sin(direction) * self.last_velocity * dt + self.last_position[1]
        self.last_position = [x, y]

        self.last_velocity += acc * dt

        # update (control) the car
        self.time_last_control = t
        self.acceleration = acc
        self.direction = direction
        self.stop = stop

        if stop:
            self.stop_time = self.last_velocity / self.acceleration + self.time_last_control

    # used with EventQueue
    # returns position of the car at specific time t
    # also more information available (but certainly not needed),f.e. acceleration, direction, steering, angle, ...
    def get_data(self, t):
        dt = 0
        if not self.stop:
            dt = (t - self.time_last_control)

        else:
            dt = (self.stop_time - self.time_last_control)

        x = 0.5 * (dt ** 2) * m.cos(self.direction) * self.acceleration + m.cos(
            self.direction) * self.last_velocity * dt + self.last_position[0]
        y = 0.5 * (dt ** 2) * m.sin(self.direction) * self.acceleration + m.sin(
            self.direction) * self.last_velocity * dt + self.last_position[1]

        return [t, self.id, round(x, 4), round(y, 4)]


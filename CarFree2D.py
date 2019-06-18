from Controller import Controller
from Path import Path
from math import sqrt, cos, sin
from Point import Point
from Polynomial import Polynomial
from PathPlanner import PathPlanner


class CarFree2D:
    def __init__(self, id: int, spawn_x, spawn_y, size_x, size_y, angle, max_vel, max_acc, color, c_dt):
        # PHYSICAL PROPERTIES
        self.color = color
        self.id = id
        self.spawn = [spawn_x, spawn_y]
        self.direction = angle
        self.position = []      # in m - later: instantiation of 2d space with dates in metres --> WAS HEISST DAS?
        self.length = size_x    # in m
        self.width = size_y     # in m
        # VELOCITY
        self.velocity = []      # in m/s
        self.max_velocity = max_vel  # [vx, vy] in m/s
        # ACCELERATION
        self.acceleration = 0  # [ax, ay] in m/s^2
        self.max_acceleration = max_acc  # [ax, ay] in m/s^2
        # STEERING
        self.steering = 0
        # PATH
        self.path_shape = []  # shape of the planned path (without exact timestamp)
        self.path = Path(self.spawn)
        self.waypoints = []     # list for the given points with the
        self.controller = Controller(self.path, self.max_acceleration, self.max_velocity, self.length)
        self.c_dt = c_dt/1000

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

    # SETTER
    #    def set_acceleration(self, ax: float, ay: float):
    #        # CHECK RANGE
    #        ax = max(-self.max_acceleration[0], min(ax, self.max_acceleration[0]))
    #        ay = max(-self.max_acceleration[1], min(ay, self.max_acceleration[1]))
    #        self.acceleration = [ax, ay]

    #    def set_velocity(self, vx: float, vy: float):
    #        # CHECK RANGE
    #        vx = max(-self.max_velocity[0], min(vx, self.max_velocity[0]))
    #        vy = max(-self.max_velocity[1], min(vy, self.max_velocity[1]))
    #        self.velocity = [vx, vy]

    def set_waypoint(self, x, y):
        p = Point(x, y)
        self.path.add(p)
        self.waypoints.append(p)
        # raise Exception('The point (' + str(p.x) + '|' + str(p.y) + ') is too far away. Skipped.')

    # SIMULATION
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

    def update2(self):
        # new Update routine

        # POSITION and VELOCITY
        x = self.spawn[0]
        y = self.spawn[1]
        t = 0
        v = 0
        for control in self.controller.controls:
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
            x += (0.5*a*self.c_dt**2 + v*self.c_dt) * cos(direction)
            y += (0.5*a*self.c_dt**2 + v*self.c_dt) * sin(direction)
            v += a * self.c_dt

        pass

    def create_spline(self):
        self.controller.calculate_controls_equidistant(self.path.points, self.c_dt)
        self.path_shape = self.controller.shape
        pass

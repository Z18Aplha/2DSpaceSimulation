from Controller import Controller
from Path import Path
from math import sqrt
from Point import Point
from Polynomial import Polynomial


class CarFree2D:
    def __init__(self, x: float, y: float, identy: int):
        # PROPERTIES
        self.color = "#00FF00"
        self.id = identy
        self.spawn = [x, y]
        self.position = []  # in m - later: instantiation of 2d space with dates in metres
        self.length = 4  # [length] = m
        self.width = 2  # [width] = m
        # VELOCITY (m/s)
        self.velocity = []
        self.max_velocity = [50.0, 10.0]  # [vx, vy]
        # ACCELERATION (DE-) (m/s^2)
        self.acceleration = []  # [ax, ay]
        self.max_acceleration = [30.0, 15.0]  # [ax, ay]
        # PATH
        self.path = Path(self.spawn)
        self.controller = Controller(self.path, self.max_acceleration, self.max_velocity)

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

    def set_destination(self, x, y, t):
        p = Point(x, y, t, True)

        if self.controller.check_possibility(self.path.points[-1], p):
            self.path.add(p)
            self.controller.create_path(self.path.points[-2], p)
        else:
            print("The point (" + str(p.x) + "|" + str(p.y) + ") is to far away. Skipped.")

    # SIMULATION
    def update(self):

        # VELOCITY
        vx = Polynomial(0, 0, 0)
        vy = Polynomial(0, 0, 0)
        t_old = 0
        velocity_x_old = 0
        velocity_y_old = 0
        for control in self.controller.controls:
            dt = control[2] - t_old
            velocity_x_old = vx.get_value(dt)
            velocity_y_old = vy.get_value(dt)
            vx = control[0].integration().add_constant(velocity_x_old)
            vy = control[1].integration().add_constant(velocity_y_old)
            t_old = control[2]
            self.velocity.append([vx, vy, control[2]])

        # POSITION
        sx = Polynomial(0, 0, self.spawn[0])
        sy = Polynomial(0, 0, self.spawn[1])
        t_old = 0
        for function in self.velocity:
            dt = function[2] - t_old
            sx_old = sx.get_value(dt)
            sy_old = sy.get_value(dt)
            sx = function[0].integration().add_constant(sx_old)
            sy = function[1].integration().add_constant(sy_old)
            t_old = function[2]
            self.position.append([sx, sy, function[2]])

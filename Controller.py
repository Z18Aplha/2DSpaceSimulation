import Lib as lib
from math import sin, cos
import random


class Controller:
    def __init__(self, car, kp, kd):
        self.k_p = kp
        self.k_d = kd
        self.car = car
        self.path = None
        self.stop_time = 0
        self.last_x_delta = 0
        self.last_y_delta = 0
        self.debug = []

    def control(self, t):
        for data in lib.data[::-1]:
            if self.car.id == data[1]:
                act_data = data
                break

        if not t > self.stop_time:
            # while True:
            #     if self.path[0][0] == t:
            #         ref_data = self.path[0]
            #         break
            #     else:
            #         self.path.pop(0)
            #         pass

            try:
                index = int(t / lib.dt)
                point = self.path[index]
            except IndexError:
                point = self.path[-1]

            x = point.real
            y = point.imag

            x_delta = act_data[2] - x
            y_delta = act_data[3] - y

            # adding some kind of noise

            # x_delta += (random.random() - 0.5) * .5
            # y_delta += (random.random() - 0.5) * .5

            dir = act_data[-1]

            vx = (x_delta - self.last_x_delta) / lib.ct
            vy = (y_delta - self.last_y_delta) / lib.ct

            ax = lib.k_p * x_delta + lib.k_d * vx
            ay = lib.k_p * y_delta + lib.k_d * vy

            self.last_x_delta = x_delta
            self.last_y_delta = y_delta

            #self.debug.append([x_delta, y_delta])

        else:
            ax, ay = 0, 0
        self.debug.append([ax,ay])
        return ax, ay

    def set_path(self, path):
        self.path = path.tolist()
        self.stop_time = self.car.planner.t_equi_in_t[-1]

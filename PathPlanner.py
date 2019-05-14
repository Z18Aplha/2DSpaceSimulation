from Point import Point
import math as m
import numpy as np
import matplotlib.pyplot as plt


def angle(p1: Point, p2: Point):
    phi = m.atan2(p2.y - p1.y, p2.x - p1.x)
    return phi


def distance(p1: Point, p2: Point):
    x = p2.x - p1.x
    y = p2.y - p1.y
    d = np.linalg.norm([x, y])
    return d


class PathPlanner:
    # based on the paper of Martin Gloderer and Andres Hertle (University of Freiburg)

    def __init__(self, points):
        self.point_list = points
        self.x = []  # just for the shape
        self.y = []  # just for the shape
        self.path = []  # shape with timestamp
        self.der1 = []
        self.der2 = []
        self.b1 = []
        self.b2 = []
        self.t_to_length = []  # plot of parameter t to length of spline
        self.dt = 0.01  # step of parameter t (0...1)

    def first_der_3(self, b1, b2):
        # returns first derivative of Bernstein Polynomial of degree 3
        x = len(b1)
        for i in range(0, len(b1)):
            der1 = []
            t = 0
            while t <= 1:
                der_x = 3 * ((b1[i][0] - self.point_list[i].x) * (1 - t) ** 2 + (b2[i][0] - b1[i][0]) * 2 * (
                        1 - t) * t + (self.point_list[i + 1].x - b2[i][0]) * (t ** 2))
                der_y = 3 * ((b1[i][1] - self.point_list[i].y) * (1 - t) ** 2 + (b2[i][1] - b1[i][1]) * 2 * (
                        1 - t) * t + (self.point_list[i + 1].y - b2[i][1]) * (t ** 2))
                der1.append([der_x, der_y])
                t += self.dt
            self.der1.append(der1)
        print('der1 done')

    def second_der_3(self, b1, b2):
        # returns second derivative of Bernstein Polynomial of degree 3
        for i in range(0, len(b1)):
            der2 = []
            t = 0
            while t <= 1:
                der_x = 6 * ((b2[i][0] - 2 * b1[i][0] + self.point_list[i].x) * (1 - t) + (
                        self.point_list[i + 1].x - 2 * b2[i][0] + b1[i][0]) * (t))
                der_y = 6 * ((b2[i][1] - 2 * b1[i][1] + self.point_list[i].y) * (1 - t) + (
                        self.point_list[i + 1].y - 2 * b2[i][1] + b1[i][1]) * (t))
                der2.append([der_x, der_y])
                t += self.dt
            self.der2.append(der2)
        print('der2 done')

    def bernstein_polynomial(self, t, x0, x1, x2, x3, y0, y1, y2, y3):
        x = (1 - t) ** 3 * x0 + 3 * (1 - t) ** 2 * t * x1 + 3 * (1 - t) * t ** 2 * \
            x2 + t ** 3 * x3

        y = (1 - t) ** 3 * y0 + 3 * (1 - t) ** 2 * t * y1 + 3 * (1 - t) * t ** 2 * \
            y2 + t ** 3 * y3

        return [x, y]

    def generate_3(self):
        # generates Bezier-Curve with Bernstein Polynomial of degree 3
        # ANGLE, TANGENT LENGTH
        phi = []
        l = []
        for i in range(0, len(self.point_list)):
            if i == 0:
                phi.append(angle(self.point_list[i], self.point_list[i + 1]))
                l.append(distance(self.point_list[i], self.point_list[i + 1]))

            if i == len(self.point_list) - 1:
                phi.append(angle(self.point_list[i - 1], self.point_list[i]))
                l.append(distance(self.point_list[i - 1], self.point_list[i]))

            if not (i == 0 or i == len(self.point_list) - 1):  # expression (2) of the paper
                phi.append(angle(self.point_list[i - 1], self.point_list[i]) + 0.5 * (
                        angle(self.point_list[i], self.point_list[i + 1]) - angle(self.point_list[i - 1],
                                                                                  self.point_list[i])))
                l.append(min(distance(self.point_list[i - 1], self.point_list[i]),
                             distance(self.point_list[i], self.point_list[i + 1])))

        # FIRST DERIVATIVE
        der1 = []
        for j in range(0, len(phi)):
            der1.append([l[j] * m.cos(phi[j]), l[j] * m.sin(phi[j])])  # [x-derivative, y-derivative]

        # CALCULATING THE CONTROL POINT FOR BEZIÉR

        for j in range(0, len(der1) - 1):  # just to len-1, last point is stop point
            self.b1.append([der1[j][0] / 3 + self.point_list[j].x, der1[j][1] / 3 + self.point_list[j].y])
            self.b2.append([-der1[j + 1][0] / 3 + self.point_list[j + 1].x, -der1[j + 1][1] / 3 + self.point_list[j + 1].y])

        # CALCULATING BEZIÉR WITH BERNSTEIN POLYNOMIAL (degree 3)
        i = 0

        while i < len(self.b1):
            section_x = []
            section_y = []
            if i == 0:
                t = 0
            else:
                t = self.dt
            while t <= 1:
                x = self.bernstein_polynomial(t, self.point_list[i].x, self.b1[i][0], self.b2[i][0], self.point_list[i + 1].x,
                                              self.point_list[i].y, self.b1[i][1], self.b2[i][1], self.point_list[i + 1].y)[0]
                y = self.bernstein_polynomial(t, self.point_list[i].x, self.b1[i][0], self.b2[i][0], self.point_list[i + 1].x,
                                              self.point_list[i].y, self.b1[i][1], self.b2[i][1], self.point_list[i + 1].y)[1]
                section_x.append(x)
                section_y.append(y)
                t += self.dt
                t = np.round(t, 7)  # (i.e. at 0.15) python does sth. like this: 0.15000000002
            self.x.append(section_x)
            self.y.append(section_y)
            i += 1

        # PLOTTING THE GIVEN CURVE
        for i in range(0, len(self.x)):
            plt.plot(self.x[i], self.y[i])

        # PLOTTING THE WAYPOINTS
        for point in self.point_list:
            plt.plot(point.x, point.y, 'bo')

        # PLOTTING THE HELP POINTS
        for point1 in self.b1:
            plt.plot(point1[0], point1[1], 'ro')
        for point2 in self.b2:
            plt.plot(point2[0], point2[1], 'co')

        # PLOTTING THE TANGENT
        for n in range(0, len(self.b1) - 1):
            plt.plot([self.b2[n][0], self.b1[n + 1][0]], [self.b2[n][1], self.b1[n + 1][1]], 'k--')

        plt.show()

        curve = [self.x, self.y]  # multiple arrays: [0:x, 1:y][n:section][i:value]

        self.first_der_3(self.b1, self.b2)
        self.second_der_3(self.b1, self.b2)

        return curve

    def get_section_length(self):
        # accuracy of calculation is limited by the chosen 'self.dt' in the generate_3 function (the higher, the better)
        length = []
        t_to_l_t = []  # just for further operations - to get an l-to-t chart
        t_to_l_l = []  # just for further operations - to get an l-to-t chart
        l_ges = 0

        for i in range(0, len(self.x)):
            l = 0
            for j in range(0, len(self.x[i]) - 1):
                dl = np.linalg.norm([self.x[i][j] - self.x[i][j + 1], self.y[i][j] - self.y[i][j + 1]])
                l += dl
                l_ges += dl
                t_to_l_t.append((i) + (j + 1) * self.dt)
                t_to_l_l.append(l_ges)
            length.append(l)

        self.t_to_length.append(t_to_l_t)
        self.t_to_length.append(t_to_l_l)
        plt.plot(self.t_to_length[0], self.t_to_length[1])
        plt.show()

        return length

    def get_curvature(self):
        # returns curvature of each calculated point
        curvature = []
        for section in range(0, len(self.point_list) - 1):
            c = []
            for i in range(0, len(self.der1[section])):
                c.append((self.der1[section][i][0] * self.der2[section][i][1] - self.der1[section][i][1] *
                          self.der2[section][i][0]) / (
                                 self.der1[section][i][0] ** 2 + self.der1[section][i][1] ** 2) ** (3 / 2))
            curvature.append(c)
        print('curvature done')
        return curvature

    def get_coordinates(self, length):

        i = 0
        x = 0
        y = 0
        t = 0
        stopped = False

        while self.t_to_length[1][i] < length and i * self.dt < self.t_to_length[1][-1]:
            i += 1

        if i+1 < len(self.t_to_length[0]):
            t = self.t_to_length[0][i] + self.dt / (
                    (self.t_to_length[1][i + 1] - self.t_to_length[1][i]) / (length - self.t_to_length[1][i]))
        else:
            t = round(self.dt*i, 0)
            stopped = True

        i = int(t)
        t -= i
        if stopped:
            x = self.point_list[-1].x
            y = self.point_list[-1].y
        else:
            x = self.bernstein_polynomial(t, self.point_list[i].x, self.b1[i][0], self.b2[i][0], self.point_list[i + 1].x,
                                          self.point_list[i].y, self.b1[i][1], self.b2[i][1], self.point_list[i + 1].y)[0]
            y = self.bernstein_polynomial(t, self.point_list[i].x, self.b1[i][0], self.b2[i][0], self.point_list[i + 1].x,
                                          self.point_list[i].y, self.b1[i][1], self.b2[i][1], self.point_list[i + 1].y)[1]

        return [x, y]

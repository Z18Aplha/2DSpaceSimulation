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


def first_der_3():
    # returns first derivative of Bernstein Polynomial of degree 3
    pass


def second_der_3():
    # returns second derivative of Bernstein Polynomial of degree 3
    pass


class PathPlanner:
    # based on the paper of Martin Gloderer and Andres Hertle (University of Freiburg)

    def __init__(self, points):
        self.point_list = points
        self.x = []
        self.y = []

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
        b1 = []
        b2 = []
        for j in range(0, len(der1) - 1):  # just to len-1, last point is stop point
            b1.append([der1[j][0] / 3 + self.point_list[j].x, der1[j][1] / 3 + self.point_list[j].y])
            b2.append([-der1[j + 1][0] / 3 + self.point_list[j + 1].x, -der1[j + 1][1] / 3 + self.point_list[j + 1].y])

        # CALCULATING BEZIÉR WITH BERNSTEIN POLYNOMIAL (degree 3)
        i = 0
        dt = 0.001  # step of parameter t (0...1)

        while i < len(b1):
            section_x = []
            section_y = []
            if i == 0:
                t = 0
            else:
                t = dt
            while t <= 1:
                x = (1 - t) ** 3 * self.point_list[i].x + 3 * (1 - t) ** 2 * t * b1[i][0] + 3 * (1 - t) * t ** 2 * \
                    b2[i][0] + t ** 3 * self.point_list[i + 1].x
                y = (1 - t) ** 3 * self.point_list[i].y + 3 * (1 - t) ** 2 * t * b1[i][1] + 3 * (1 - t) * t ** 2 * \
                    b2[i][1] + t ** 3 * self.point_list[i + 1].y
                section_x.append(x)
                section_y.append(y)
                t += dt
                t = np.round(t, 7)  # don't know why, but (i.e. at 0.15) python does sth. like this: 0.15000000002
            self.x.append(section_x)
            self.y.append(section_y)
            i += 1

        # PLOTTING THE GIVEN CURVE
        for i in range(0, len(self.x)):
            plt.plot(self.x[i], self.y[i])

        # PLOTTING THE GIVEN WAYPOINTS
        for point in self.point_list:
            plt.plot(point.x, point.y, 'bo')

        # PLOTTING THE HELP POINTS
        for point1 in b1:
            plt.plot(point1[0], point1[1], 'ro')
        for point2 in b2:
            plt.plot(point2[0], point2[1], 'co')

        # PLOTTING THE TANGENT
        for n in range(0, len(b1) - 1):
            plt.plot([b2[n][0], b1[n + 1][0]], [b2[n][1], b1[n + 1][1]], 'k--')

        #plt.show()

        curve = [self.x, self.y]  # multiple arrays: [0:x, 1:y][n:section][i:value]

        return curve

    def get_section_length(self):
        # accuracy of calculation is limited by the chosen 'dt' in the generate_3 function (the higher, the better)
        length = []
        for i in range(0, len(self.x)):
            l = 0
            for j in range(0, len(self.x[i]) - 1):
                l += np.linalg.norm([self.x[i][j] - self.x[i][j + 1], self.y[i][j] - self.y[i][j + 1]])
            length.append(l)

        return length

    def get_curvature(self):
        # returns curvature of each calculated point
        pass

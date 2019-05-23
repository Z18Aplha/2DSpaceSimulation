import pyglet
from pyglet.gl import *
from God import God
from matplotlib import colors
import math


class SpaceFree2DOpenGL(pyglet.window.Window):

    def __init__(self, g: God):
        self.height1 = int(g.size[1]+3)
        self.width1 = int(g.size[0]+3)
        self.px_width = int(g.parameters["SpaceFree2D"]["px_width"])
        self.px_height = int(self.height1 / self.width1 * self.px_width)
        self.pxm = self.px_width / self.width1  # pixel per meter
        self.g = g
        self.car_models = []
        self.car_rect = []
        self.obstacles = []
        self.coordinates = []
        self.timestamp = 0
        self.dataset = g.calculation[:]
        self.start = True
        self.counter = 0
        self.fps = 50
        super().__init__(width=self.px_width, height=self.px_height, visible=True)

    def on_draw(self):
        for obs in self.g.obstacles:
            glColor3f(colors.to_rgb(obs.color)[0], colors.to_rgb(obs.color)[1], colors.to_rgb(obs.color)[2])
            glBegin(GL_QUADS)
            glVertex2f(obs.edges[0] * self.pxm, obs.edges[1] * self.pxm)
            glVertex2f(obs.edges[2] * self.pxm, obs.edges[3] * self.pxm)
            glVertex2f(obs.edges[4] * self.pxm, obs.edges[5] * self.pxm)
            glVertex2f(obs.edges[6] * self.pxm, obs.edges[7] * self.pxm)
            glEnd()
            for car in self.g.cars:
                if self.start:
                    x1 = (car.spawn[0] - car.length / 2) * self.pxm
                    y1 = (car.spawn[1] - car.width / 2) * self.pxm
                    x2 = (car.spawn[0] + car.length / 2) * self.pxm
                    y2 = (car.spawn[1] - car.width / 2) * self.pxm
                    x3 = (car.spawn[0] + car.length / 2) * self.pxm
                    y3 = (car.spawn[1] + car.width / 2) * self.pxm
                    x4 = (car.spawn[0] - car.length / 2) * self.pxm
                    y4 = (car.spawn[1] + car.width / 2) * self.pxm
                    self.coordinates.append([x1, y1, x2, y2, x3, y3, x4, y4])
                try:
                    glColor3f(colors.to_rgb(car.color)[0], colors.to_rgb(car.color)[1], colors.to_rgb(car.color)[2])
                except ValueError:
                    clr = self.hex_to_rgb(car.color)
                    glColor3f(clr[0], clr[1], clr[2])
                glBegin(GL_QUADS)
                i = self.g.cars.index(car)
                glVertex2f(self.coordinates[i][0], self.coordinates[i][1])
                glVertex2f(self.coordinates[i][2], self.coordinates[i][3])
                glVertex2f(self.coordinates[i][4], self.coordinates[i][5])
                glVertex2f(self.coordinates[i][6], self.coordinates[i][7])
                glEnd()

                # Wheels
                radius = min(car.length, car.width) * self.pxm / 6
                self.circle(self.coordinates[i][0] + 1.5 * radius, self.coordinates[i][1] + 1.5 * radius, radius)
                self.circle(self.coordinates[i][2] - 1.5 * radius, self.coordinates[i][3] + 1.5 * radius, radius)
                self.circle(self.coordinates[i][4] - 1.5 * radius, self.coordinates[i][5] - 1.5 * radius, radius)
                self.circle(self.coordinates[i][6] + 1.5 * radius, self.coordinates[i][7] - 1.5 * radius, radius)
        self.show_waypoints()
        self.show_interpolated_shape()
        if self.timestamp <= self.g.last_timestamp:
            pyglet.image.get_buffer_manager().get_color_buffer().save('video/' + str(self.counter) + '.jpg')
            self.counter += 1

    def circle(self, x, y, radius):
        iterations = int(2 * radius * math.pi)
        s = math.sin(2 * math.pi / iterations)
        c = math.cos(2 * math.pi / iterations)

        dx, dy = radius, 0
        glColor3f(1, 1, 1)
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(x, y)
        for i in range(iterations + 1):
            glVertex2f(x + dx, y + dy)
            dx, dy = (dx * c - dy * s), (dy * c + dx * s)
        glEnd()

    def show_waypoints(self):
        for car in self.g.cars:
            for point in car.waypoints:
                size = 0.25 * self.pxm
                try:
                    glColor3f(colors.to_rgb(car.color)[0], colors.to_rgb(car.color)[1], colors.to_rgb(car.color)[2])
                except ValueError:
                    clr = self.hex_to_rgb(car.color)
                    glColor3f(clr[0], clr[1], clr[2])
                glBegin(GL_QUADS)
                glVertex2f(point.x*self.pxm-size, point.y*self.pxm-size)
                glVertex2f(point.x*self.pxm+size, point.y*self.pxm-size)
                glVertex2f(point.x*self.pxm + size, point.y*self.pxm + size)
                glVertex2f(point.x*self.pxm - size, point.y*self.pxm + size)
                glEnd()

    def show_interpolated_shape(self):
        for car in self.g.cars:
            sections = len(car.path_shape[0])
            x_old = car.spawn[0]
            y_old = car.spawn[1]
            for n in range(0, sections):
                number_of_values = len(car.path_shape[0][n])
                for i in range(0, number_of_values):
                    x = car.path_shape[0][n][i]
                    y = car.path_shape[1][n][i]
                    try:
                        glColor3f(colors.to_rgb(car.color)[0], colors.to_rgb(car.color)[1], colors.to_rgb(car.color)[2])
                    except ValueError:
                        clr = self.hex_to_rgb(car.color)
                        glColor3f(clr[0], clr[1], clr[2])
                    glBegin(GL_LINES)
                    glVertex2f(x_old * self.pxm, y_old * self.pxm)
                    glVertex2f(x * self.pxm, y * self.pxm)
                    glEnd()
                    x_old = x
                    y_old = y

    def show_shape(self):
        for car in self.g.cars:
            sections = len(car.path_shape[0])
            for n in range(0, sections):
                number_of_values = len(car.path_shape[0][n])
                for i in range(0, number_of_values):
                    x = car.path_shape[0][n][i]
                    y = car.path_shape[1][n][i]
                    try:
                        glColor3f(colors.to_rgb(car.color)[0], colors.to_rgb(car.color)[1], colors.to_rgb(car.color)[2])
                    except ValueError:
                        clr = self.hex_to_rgb(car.color)
                        glColor3f(clr[0], clr[1], clr[2])
                    glBegin(GL_POINTS)
                    glVertex2f(x * self.pxm, y * self.pxm)
                    glEnd()

    def update(self, dt):
        if self.timestamp > self.g.last_timestamp:
            pyglet.clock.unschedule(self.update)
            pyglet.app.exit()
        self.clear()
        self.start = False
        i = 0
        while i < len(self.dataset):
            data = self.dataset[i]
            if data[1] == self.timestamp:
                car = self.g.cars[data[0]]
                x1 = (data[2] - car.length / 2) * self.pxm
                y1 = (data[3] - car.width / 2) * self.pxm
                x2 = (data[2] + car.length / 2) * self.pxm
                y2 = (data[3] - car.width / 2) * self.pxm
                x3 = (data[2] + car.length / 2) * self.pxm
                y3 = (data[3] + car.width / 2) * self.pxm
                x4 = (data[2] - car.length / 2) * self.pxm
                y4 = (data[3] + car.width / 2) * self.pxm
                self.coordinates[data[0]] = [x1, y1, x2, y2, x3, y3, x4, y4]
                self.dataset.remove(data)
            else:
                if data[1] < self.timestamp:
                    i += 1
                else:
                    break
        self.timestamp = round(self.timestamp+(1/self.fps), 3)

    def create_space(self):
        print(self.g.last_timestamp)
        pyglet.clock.schedule_interval(self.update, 1/self.fps)
        pyglet.app.run()


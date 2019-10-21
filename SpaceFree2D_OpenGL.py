import pyglet
from pyglet.gl import *
from God import God
from matplotlib import colors
import math
import time
from pyglet import clock
import imageio
import Lib as lib
import numpy as np
import os as os

class SpaceFree2DOpenGL(pyglet.window.Window):

    def __init__(self, g: God):
        self.height1 = int(g.size[1])
        self.width1 = int(g.size[0])
        self.px_width = int(g.parameters["SpaceFree2D"]["px_width"])
        self.px_height = int(self.height1 / self.width1 * self.px_width)
        self.pxm = self.px_width / self.width1  # pixel per meter
        self.g = g
        self.car_models = []
        self.car_rect = []
        self.front = []
        self.back = []
        for cars in g.cars:
            self.front.append('yellow')
            self.back.append('red')
        self.obstacles = []
        self.coordinates = []
        for car in lib.carList:
            self.coordinates.append([])
        self.timestamp = 0
        self.dataset = lib.vis_data[:]
        self.start = True
        self.stop = g.last_timestamp
        self.counter = 0
        self.blink_counter = 0
        self.fps = lib.fps
        super().__init__(caption="AVG Simulator", width=self.px_width, height=self.px_height, visible=True)
        self.time = []
        self.currentfps = []
        self.collided = False
        self.running = True
        self.animSprite = 0

    def on_draw(self):
        pyglet.gl.glEnable(pyglet.gl.GL_BLEND)
        pyglet.gl.glBlendFunc(pyglet.gl.GL_SRC_ALPHA, pyglet.gl.GL_ONE_MINUS_SRC_ALPHA)

        if self.running:
            if self.collided:
                if self.blink_counter >= (self.fps / 5):
                    self.blink_counter = 0
                    if self.front[self.g.collisions[1]] == 'orange':
                        self.front[self.g.collisions[1]] = 'yellow'
                        self.front[self.g.collisions[2]] = 'yellow'
                        self.back[self.g.collisions[1]] = 'red'
                        self.back[self.g.collisions[2]] = 'red'
                    else:
                        self.front[self.g.collisions[1]] = 'orange'
                        self.front[self.g.collisions[2]] = 'orange'
                        self.back[self.g.collisions[1]] = 'orange'
                        self.back[self.g.collisions[2]] = 'orange'

                self.blink_counter += 1

            t1 = time.time()
            # self.show_interpolated_shape()
            # self.show_shape()
            # self.show_shape_optimized()
            self.show_shape_optimized_new()
            for obs in self.g.obstacles:
                glColor3f(colors.to_rgb(obs.color)[0], colors.to_rgb(obs.color)[1], colors.to_rgb(obs.color)[2])
                glBegin(GL_QUADS)
                glVertex2f(obs.edges[0] * self.pxm, obs.edges[1] * self.pxm)
                glVertex2f(obs.edges[2] * self.pxm, obs.edges[3] * self.pxm)
                glVertex2f(obs.edges[4] * self.pxm, obs.edges[5] * self.pxm)
                glVertex2f(obs.edges[6] * self.pxm, obs.edges[7] * self.pxm)
                glEnd()
            self.show_waypoints()

            for car in self.g.cars:
                if car.ghost:
                    transparancy = .4
                else:
                    transparancy = 1
                i = self.g.cars.index(car)
                if self.start:
                    x0 = car.spawn[0]
                    y0 = car.spawn[1]
                    angle = car.start_direction
                    x1 = car.spawn[0] - car.length / 2
                    y1 = car.spawn[1] - car.width / 2
                    x2 = car.spawn[0] + car.length / 2
                    y2 = car.spawn[1] - car.width / 2
                    x3 = car.spawn[0] + car.length / 2
                    y3 = car.spawn[1] + car.width / 2
                    x4 = car.spawn[0] - car.length / 2
                    y4 = car.spawn[1] + car.width / 2
                    self.coordinates[lib.carList.index(car)] = [x1, y1, x2, y2, x3, y3, x4, y4, x0, y0, angle]
                try:
                    glColor4f(colors.to_rgb(car.color)[0], colors.to_rgb(car.color)[1], colors.to_rgb(car.color)[2], transparancy)
                except ValueError:
                    clr = self.hex_to_rgb(car.color)
                    glColor4f(clr[0], clr[1], clr[2], transparancy)
                angle = self.coordinates[i][-1]
                x0 = self.coordinates[i][-3]
                y0 = self.coordinates[i][-2]
                glBegin(GL_QUADS)
                x, y = self.rotate(self.coordinates[i][0], self.coordinates[i][1], x0, y0, angle)
                glVertex2f(x, y)  # x1, y1
                x, y = self.rotate(self.coordinates[i][2], self.coordinates[i][3], x0, y0, angle)
                glVertex2f(x, y)  # x2, y2
                x, y = self.rotate(self.coordinates[i][4], self.coordinates[i][5], x0, y0, angle)
                glVertex2f(x, y)  # x3, y3
                x, y = self.rotate(self.coordinates[i][6], self.coordinates[i][7], x0, y0, angle)
                glVertex2f(x, y)  # x4, y4
                glEnd()

                # Wheels
                radius = min(car.length, car.width) / 6
                x, y = self.rotate(self.coordinates[i][0] + 1.5 * radius, self.coordinates[i][1] + 1.5 * radius, x0, y0,
                                   angle)
                self.circle(x, y, radius * self.pxm, transparancy)
                x, y = self.rotate(self.coordinates[i][2] - 1.5 * radius, self.coordinates[i][3] + 1.5 * radius, x0, y0,
                                   angle)
                self.circle(x, y, radius * self.pxm, transparancy)
                x, y = self.rotate(self.coordinates[i][4] - 1.5 * radius, self.coordinates[i][5] - 1.5 * radius, x0, y0,
                                   angle)
                self.circle(x, y, radius * self.pxm, transparancy)
                x, y = self.rotate(self.coordinates[i][6] + 1.5 * radius, self.coordinates[i][7] - 1.5 * radius, x0, y0,
                                   angle)
                self.circle(x, y, radius * self.pxm, transparancy)

                x, y = self.rotate(self.coordinates[i][0]+(self.coordinates[i][2]-self.coordinates[i][0])/2,
                                   self.coordinates[i][1]+(self.coordinates[i][7]-self.coordinates[i][1])/2, x0, y0,
                                   angle)
                pyglet.text.Label(str(car.id), font_name='Times New Roman', font_size=10,
                                  x=x,
                                  y=y,
                                  anchor_x='center', anchor_y='center').draw()

                # Lights
                size = (car.width / 10)
                # Front
                glColor4f(colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], transparancy)
                glBegin(GL_QUADS)
                x, y = self.rotate(self.coordinates[i][2] - size / 2, self.coordinates[i][5]-2*size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][2] + size / 2, self.coordinates[i][5]-2*size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][2] + size / 2, self.coordinates[i][5]-size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][2] - size / 2, self.coordinates[i][5]-size, x0, y0, angle)
                glVertex2f(x, y)
                glEnd()

                # Lightbeams
                x1, y1 = self.rotate(self.coordinates[i][2] + size / 2, self.coordinates[i][5]-2*size, x0, y0, angle)
                x2, y2 = self.rotate(self.coordinates[i][2] + 5*size, self.coordinates[i][5]-5*size, x0, y0, angle)
                x3, y3 = self.rotate(self.coordinates[i][2] + 5*size, self.coordinates[i][5]+2*size, x0, y0, angle)
                x4, y4 = self.rotate(self.coordinates[i][2] + size / 2, self.coordinates[i][5]-size, x0, y0, angle)
                pyglet.graphics.draw(4, pyglet.gl.GL_QUADS,
                                     ('v2f', (x1, y1, x2, y2, x3, y3, x4, y4)
                                      ),
                                     ('c4f', (colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], transparancy,
                                              colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], 0,
                                              colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], 0,
                                              colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], transparancy)
                                      )
                                     )
                glBegin(GL_QUADS)
                x, y = self.rotate(self.coordinates[i][2] - size / 2, self.coordinates[i][3] + 2 * size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][2] + size / 2, self.coordinates[i][3] + 2 * size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][2] + size / 2, self.coordinates[i][3] + size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][2] - size / 2, self.coordinates[i][3] + size, x0, y0, angle)
                glVertex2f(x, y)
                glEnd()

                x1, y1 = self.rotate(self.coordinates[i][2] + size / 2, self.coordinates[i][3] + 2 * size, x0, y0, angle)
                x2, y2 = self.rotate(self.coordinates[i][2] + 5 * size, self.coordinates[i][3] + 5 * size, x0, y0, angle)
                x3, y3 = self.rotate(self.coordinates[i][2] + 5 * size, self.coordinates[i][3] - 2 * size, x0, y0, angle)
                x4, y4 = self.rotate(self.coordinates[i][2] + size / 2, self.coordinates[i][3] + size, x0, y0, angle)

                pyglet.graphics.draw(4, pyglet.gl.GL_QUADS,
                                     ('v2f', (x1, y1, x2, y2, x3, y3, x4, y4)
                                      ),
                                     ('c4f', (colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], transparancy,
                                              colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], 0,
                                              colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], 0,
                                              colors.to_rgb(self.front[i])[0], colors.to_rgb(self.front[i])[1], colors.to_rgb(self.front[i])[2], transparancy)
                                      )
                                     )
                # Back
                glColor4f(colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1], colors.to_rgb(self.back[i])[2], transparancy)
                glBegin(GL_QUADS)
                x, y = self.rotate(self.coordinates[i][0] + size / 2, self.coordinates[i][5] - 2 * size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][0] - size / 2, self.coordinates[i][5] - 2 * size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][0] - size / 2, self.coordinates[i][5] - size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][0] + size / 2, self.coordinates[i][5] - size, x0, y0, angle)
                glVertex2f(x, y)
                glEnd()

                x1, y1 = self.rotate(self.coordinates[i][0] - size / 2, self.coordinates[i][5] - 2 * size, x0, y0, angle)
                x2, y2 = self.rotate(self.coordinates[i][0] - size / 2, self.coordinates[i][5] - size, x0, y0, angle)
                x3, y3 = self.rotate(self.coordinates[i][0] - 5 * size, self.coordinates[i][5] + 2 * size, x0, y0, angle)
                x4, y4 = self.rotate(self.coordinates[i][0] - 5 * size, self.coordinates[i][5] - 5 * size, x0, y0, angle)
                pyglet.graphics.draw(4, pyglet.gl.GL_QUADS,
                                     ('v2f', (x1, y1, x2, y2, x3, y3, x4, y4)
                                      ),
                                     ('c4f', (colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1],
                                              colors.to_rgb(self.back[i])[2], transparancy,
                                              colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1],
                                              colors.to_rgb(self.back[i])[2], transparancy,
                                              colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1],
                                              colors.to_rgb(self.back[i])[2], 0,
                                              colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1],
                                              colors.to_rgb(self.back[i])[2], 0)
                                      )
                                     )
                glBegin(GL_QUADS)
                x, y = self.rotate(self.coordinates[i][0] + size / 2, self.coordinates[i][3] + 2 * size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][0] - size / 2, self.coordinates[i][3] + 2 * size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][0] - size / 2, self.coordinates[i][3] + size, x0, y0, angle)
                glVertex2f(x, y)
                x, y = self.rotate(self.coordinates[i][0] + size / 2, self.coordinates[i][3] + size, x0, y0, angle)
                glVertex2f(x, y)
                glEnd()
                x1, y1 = self.rotate(self.coordinates[i][0] - size / 2, self.coordinates[i][3] + 2 * size, x0, y0, angle)
                x2, y2 = self.rotate(self.coordinates[i][0] - size / 2, self.coordinates[i][3] + size, x0, y0, angle)
                x3, y3 = self.rotate(self.coordinates[i][0] - 5 * size, self.coordinates[i][3] - 2 * size, x0, y0, angle)
                x4, y4 = self.rotate(self.coordinates[i][0] - 5 * size, self.coordinates[i][3] + 5 * size, x0, y0, angle)
                pyglet.graphics.draw(4, pyglet.gl.GL_QUADS,
                                     ('v2f', (x1, y1, x2, y2, x3, y3, x4, y4)
                                      ),
                                     ('c4f', (colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1],
                                              colors.to_rgb(self.back[i])[2], transparancy,
                                              colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1],
                                              colors.to_rgb(self.back[i])[2], transparancy,
                                              colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1],
                                              colors.to_rgb(self.back[i])[2], 0,
                                              colors.to_rgb(self.back[i])[0], colors.to_rgb(self.back[i])[1],
                                              colors.to_rgb(self.back[i])[2], 0)
                                      )
                                     )

            label = pyglet.text.Label(str(self.timestamp),
                                      font_name='Times New Roman',
                                      font_size=10,
                                      x=3, y=3,
                                      anchor_x='left', anchor_y='bottom')
            label.draw()

            # while animating save current frame
            if self.timestamp <= self.g.last_timestamp:
                pyglet.image.get_buffer_manager().get_color_buffer().save('video/' + str(self.counter) + '.png')
                self.counter += 1
            t2 = time.time()
            self.time.append(t2-t1)
            self.currentfps.append(clock.get_fps())

            pyglet.text.Label("Animating",
                              font_name='Times New Roman',
                              font_size=10,
                              x=self.width - 80, y=3,
                              anchor_x='left', anchor_y='bottom').draw()
        else:
            self.clear()
            self.animSprite.draw()

    def on_mouse_press(self, x, y, button, modifiers):
        print(round(x / self.pxm, 2), round(y/self.pxm, 2))

    def circle(self, x, y, radius, t):
        iterations = int(2 * radius * math.pi)
        s = math.sin(2 * math.pi / iterations)
        c = math.cos(2 * math.pi / iterations)

        dx, dy = radius, 0
        glColor4f(1, 1, 1, t)
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(x, y)
        for i in range(iterations + 1):
            glVertex2f(x + dx, y + dy)
            dx, dy = (dx * c - dy * s), (dy * c + dx * s)
        glEnd()

    def rotate(self, x, y, x0, y0, angle):
        x = x - x0
        y = y - y0
        xx = x * np.cos(angle) - y * np.sin(angle)
        yy = x * np.sin(angle) + y * np.cos(angle)

        x_new = xx + x0
        y_new = yy + y0
        return x_new * self.pxm, y_new * self.pxm

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
            sections = len(car.shape[0])
            x_old = car.spawn[0]
            y_old = car.spawn[1]
            for n in range(0, sections):
                number_of_values = len(car.shape[0][n])
                for i in range(0, number_of_values):
                    x = car.shape[0][n][i]
                    y = car.shape[1][n][i]
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
            sections = len(car.shape[0])
            for n in range(0, sections):
                number_of_values = len(car.shape[0][n])
                for i in range(0, number_of_values):
                    x = car.shape[0][n][i]
                    y = car.shape[1][n][i]
                    try:
                        glColor3f(colors.to_rgb(car.color)[0], colors.to_rgb(car.color)[1], colors.to_rgb(car.color)[2])
                    except ValueError:
                        clr = self.hex_to_rgb(car.color)
                        glColor3f(clr[0], clr[1], clr[2])
                    glBegin(GL_POINTS)
                    glVertex2f(x * self.pxm, y * self.pxm)
                    glEnd()

    def show_shape_optimized(self):
        if self.start:
            for car in self.g.cars:
                if car.ghost:
                    transparancy = .4
                else:
                    transparancy = 255
                sections = len(car.shape[0])
                x_old = car.spawn[0]
                y_old = car.spawn[1]
                for n in range(0, sections):
                    c = car.shape[0]
                    number_of_values = len(car.shape[0][n])
                    for i in range(0, number_of_values):
                        x = car.shape[0][n][i]
                        y = car.shape[1][n][i]
                        try:
                            glColor4f(colors.to_rgb(car.color)[0], colors.to_rgb(car.color)[1],
                                      colors.to_rgb(car.color)[2], transparancy)
                        except ValueError:
                            clr = self.hex_to_rgb(car.color)
                            glColor4f(clr[0], clr[1], clr[2], transparancy)
                        glBegin(GL_LINES)
                        glVertex2f(x_old * self.pxm, y_old * self.pxm)
                        glVertex2f(x * self.pxm, y * self.pxm)
                        glEnd()
                        x_old = x
                        y_old = y
            pyglet.image.get_buffer_manager().get_color_buffer().save('video/background.png')
        pyglet.gl.glClearColor(0, 0, 0, 0)
        image = pyglet.image.load("video/background.png")
        sprite = pyglet.sprite.Sprite(image, 0, 0)
        sprite.draw()

    def show_shape_optimized_new(self):
        if self.start:
            for car in self.g.cars:
                if car.ghost:
                    transparancy = .4
                else:
                    transparancy = 255
                x_old = car.spawn[0]
                y_old = car.spawn[1]

                for point in car.shape:
                    x = point[0]
                    y = point[1]
                    try:
                        glColor4f(colors.to_rgb(car.color)[0], colors.to_rgb(car.color)[1],
                                  colors.to_rgb(car.color)[2], transparancy)
                    except ValueError:
                        clr = self.hex_to_rgb(car.color)
                        glColor4f(clr[0], clr[1], clr[2], transparancy)
                    glBegin(GL_LINES)
                    glVertex2f(x_old * self.pxm, y_old * self.pxm)
                    glVertex2f(x * self.pxm, y * self.pxm)
                    glEnd()
                    x_old = x
                    y_old = y
            try:
                os.remove('video/background.png')
            except FileNotFoundError:
                pass
            pyglet.image.get_buffer_manager().get_color_buffer().save('video/background.png')
        pyglet.gl.glClearColor(0, 0, 0, 0)
        try:
            image = pyglet.image.load("video/background.png")
            sprite = pyglet.sprite.Sprite(image, 0, 0)
            sprite.draw()
        except FileNotFoundError:
            self.start = True
            self.show_shape_optimized_new()

    def update(self, dt):
        if self.timestamp > self.stop:
            pyglet.clock.unschedule(self.update)
            im = 0
            writer = imageio.get_writer('animation.mp4', fps=self.fps)
            for i in range(self.counter):
                im = imageio.imread('video/' + str(i) + '.png')
                writer.append_data(im)
            for i in range(self.fps):
                writer.append_data(im)
            writer.close()

            # animation = pyglet.image.load_animation('animation.mp4')
            # self.animSprite = pyglet.sprite.Sprite(animation)
            self.running = False
            pyglet.app.exit()
        self.clear()
        self.start = False
        if self.timestamp <= self.g.collisions[0]:
            i = 0
            while i < len(self.dataset):
                data = self.dataset[i]
                if data[0] == self.timestamp:
                    try:
                        car = self.g.cars[data[1]]
                    except TypeError:
                        for c in self.g.cars[::-1]:
                            if c.id == data[1]:
                                car = c
                    x1 = data[2] - car.length / 2
                    y1 = data[3] - car.width / 2
                    x2 = data[2] + car.length / 2
                    y2 = data[3] - car.width / 2
                    x3 = data[2] + car.length / 2
                    y3 = data[3] + car.width / 2
                    x4 = data[2] - car.length / 2
                    y4 = data[3] + car.width / 2

                    angle = data[-1]
                    lib.carList.index(car)
                    self.coordinates[lib.carList.index(car)] = [x1, y1, x2, y2, x3, y3, x4, y4, data[2], data[3], angle]
                    self.dataset.remove(data)
                else:
                    if data[0] < self.timestamp:
                        i += 1
                    else:
                        if self.dataset[i-1][0] < self.timestamp:
                            data = self.dataset[i-1]
                            try:
                                car = self.g.cars[data[1]]
                            except TypeError:
                                for c in self.g.cars[::-1]:
                                    if c.id == data[1]:
                                        car = c
                            x1 = data[2] - car.length / 2
                            y1 = data[3] - car.width / 2
                            x2 = data[2] + car.length / 2
                            y2 = data[3] - car.width / 2
                            x3 = data[2] + car.length / 2
                            y3 = data[3] + car.width / 2
                            x4 = data[2] - car.length / 2
                            y4 = data[3] + car.width / 2

                            angle = data[-1]

                            self.coordinates[lib.carList.index(car)] = [x1, y1, x2, y2, x3, y3, x4, y4, data[2], data[3], angle]
                        break

        else:
            self.stop = self.g.collisions[0]+3
            self.collided = True
        self.timestamp = round(self.timestamp + 1/self.fps, 3)

    def create_space(self):
        print(f"\n max. animation duration: {min(self.g.last_timestamp, self.g.collisions[0]):< 1.4} seconds")
        pyglet.clock.schedule_interval(self.update, 1/self.fps)
        pyglet.app.run()

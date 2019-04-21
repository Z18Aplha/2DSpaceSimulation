from tkinter import *
from God import God
from CarFree2D import CarFree2D
import time
from Obstacles2D import Obstacles2D



class SpaceSimulation2D:
    def __init__(self, g: God):
        self.height = g.size[1]+3
        self.width = g.size[0]+3
        self.px_width = 600
        self.px_height = self.height/self.width * self.px_width
        # self.px_height = 500
        # self.width = self.height * self.px_width / self.px_height
        self.pxm = self.px_width / self.width  # pixel per meter
        self.g = g
        self.car_rect = []
        self.obstacles = []
        # WINDOW
        self.window = Tk()
        self.grid = Canvas(bg="black", height=self.px_height, width=self.px_width)
        self.button_start = Button(self.window, text="Start animation!", command=self.show,
                                   width=int(self.px_width / 22))
        self.label_status = Label(self.window, text="waiting for start", anchor="e", width=int(self.px_width / 22))
        self.label_time = Label(self.window, text="", anchor="w", width=int(self.px_width / 22))
        # self.grid.create_line(100, 100, 500, 500, fill="red")

        self.grid.grid(row=0, column=0, columnspan=3)
        self.button_start.grid(row=1, column=1, sticky="e")
        self.label_status.grid(row=1, column=2, sticky="e")
        self.label_time.grid(row=1, column=0, sticky="w")

        # scalable - width/pixels = metres/Pixel
        # later: obstacles

    def create_space(self):
        for car in self.g.cars:
            rect = self.grid.create_rectangle((car.spawn[0] - car.length/2) * self.pxm, (car.spawn[1] - car.width/2) * self.pxm,
                                              (car.spawn[0] + car.length/2) * self.pxm,
                                              (car.spawn[1] + car.width/2) * self.pxm, fill=car.color),
            self.car_rect.append([car.id, rect, car.length, car.width])

        for obst in self.g.obstacles:
            spawn = []
            for i in obst.edges:
                spawn.append(i*self.pxm)
            obstrect = self.grid.create_polygon(spawn, fill=obst.color),
            self.obstacles.append([obstrect])

        self.window.mainloop()

    def show_path(self):
        for car in self.g.cars:
            length = len(car.path.points)
            x_old = car.spawn[0]
            y_old = car.spawn[1]
            for i in range(0, length):
                x = car.path.points[i].x
                y = car.path.points[i].y
                self.grid.create_line(x_old * self.pxm, y_old * self.pxm, x * self.pxm, y * self.pxm, fill=car.color)
                x_old = x
                y_old = y

    def show(self):
        self.show_path()
        t = 0
        self.label_status["text"] = "animating..."
        for data in self.g.calculation:
            sleep = data[1] - t
            t = data[1]
            time.sleep(sleep)
            for shape in self.car_rect:
                if data[0] == shape[0]:
                    self.grid.coords(shape[1], (data[2] - shape[2]/2)* self.pxm, (data[3] - shape[3]/2) * self.pxm, (data[2] + shape[2]/2) * self.pxm,
                                     (data[3] + shape[3]/2) * self.pxm)
                    self.label_time["text"] = t
                    self.window.update()
        self.label_status["text"] = "animation done!"
        self.window.update()

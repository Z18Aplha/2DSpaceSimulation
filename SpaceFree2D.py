from tkinter import *
from God import God
from CarFree2D import CarFree2D
import time
from Obstacles2D import Obstacles2D


class SpaceSimulation2D:
    def __init__(self, g: God):
        self.height = g.size[1] + 3
        self.width = g.size[0] + 3
        self.px_width = g.parameters["SpaceFree2D"]["px_width"]
        self.px_height = self.height / self.width * self.px_width
        # self.px_height = 500
        # self.width = self.height * self.px_width / self.px_height
        self.pxm = self.px_width / self.width  # pixel per meter
        self.g = g
        #### MERGE ####
        self.car_models = []
        self.car_rect = []
        self.obstacles = []
        ###############

        # WINDOW
        self.window = Tk()
        self.grid = Canvas(bg="black", height=self.px_height, width=self.px_width)
        self.button_start = Button(self.window, text="Start animation!", command=self.show2,
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
            # (x1,y1) and (x2,y2) are the points of the diagonal of the car model (rectangle)
            x1 = (car.spawn[0] - car.length / 2) * self.pxm
            y1 = (car.spawn[1] - car.width / 2) * self.pxm
            x2 = (car.spawn[0] + car.length / 2) * self.pxm
            y2 = (car.spawn[1] + car.width / 2) * self.pxm

            # tag is used for group rect and wheels (to stick them together)
            # attention: turns will be a problem - car is no 'picture' what can be turned
            tag = 'car'
            tag += str(car.id)   # example: car0, car1

            rect = self.grid.create_rectangle(x1, y1, x2, y2, fill=car.color, tags = tag)

            # Wheels for each car (circles, because CarFree2D)
            radius = min(car.length, car.width)*self.pxm/6
            wheel1 = self.grid.create_oval(x1+radius/2, y1+ radius/2, x1+2.5*radius, y1+2.5*radius, fill='white', tags = tag)
            wheel2 = self.grid.create_oval(x2-radius/2, y1+ radius/2, x2-2.5*radius, y1+2.5*radius, fill='white', tags = tag)
            wheel3 = self.grid.create_oval(x2-radius/2, y2- radius/2, x2-2.5*radius, y2-2.5*radius, fill='white', tags = tag)
            wheel4 = self.grid.create_oval(x1+radius/2, y2- radius/2, x1+2.5*radius, y2-2.5*radius, fill='white', tags = tag)
            arrow = self.grid.create_text(car.spawn[0]*self.pxm, car.spawn[1]*self.pxm, text = '-->', tags = tag)

            #self.car_models.append([car.id + 300, wheel1, 2, 2])
            #self.car_models.append([car.id, rect, car.length, car.width])
            #self.car_models.append([car.id, tag])

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

    def show_waypoints(self):
        for car in self.g.cars:
            for point in car.waypoints:
                size = 0.25*self.pxm
                self.grid.create_rectangle(point.x*self.pxm-size, point.y*self.pxm-size, point.x*self.pxm + size, point.y*self.pxm + size, fill=car.color)

    def show_shape(self):
        # shows the calculated shape of the car (beziér curve)
        for car in self.g.cars:
            sections = len(car.path_shape[0])
            x_old = car.spawn[0]
            y_old = car.spawn[1]
            for n in range(0, sections):
                number_of_values = len(car.path_shape[0][n])
                for i in range(0, number_of_values):
                    x = car.path_shape[0][n][i]
                    y = car.path_shape[1][n][i]
                    self.grid.create_line(x_old * self.pxm, y_old * self.pxm, x * self.pxm, y * self.pxm,
                                          fill=car.color)
                    x_old = x
                    y_old = y

    def show(self):
        self.show_path()
        self.show_shape()
        self.show_waypoints()
        t = 0
        self.label_status["text"] = "animating..."
        for data in self.g.calculation:
            sleep = data[1] - t
            t = data[1]
            time.sleep(sleep)
            for shape in self.car_models:
                if data[0] == shape[0]:
                    self.grid.coords(shape[1], (data[2] - shape[2] / 2) * self.pxm, (data[3] - shape[3] / 2) * self.pxm,
                                     (data[2] + shape[2] / 2) * self.pxm,
                                     (data[3] + shape[3] / 2) * self.pxm)
                    self.label_time["text"] = t
                    self.window.update()
        self.label_status["text"] = "animation done!"
        self.window.update()

    def show2(self):
        #self.show_path()
        self.show_shape()
        self.show_waypoints()
        self.label_status["text"] = "animating..."
        # storage for prior point, list index i is equal to the car_id (if car id's are correct in Parameters.json)
        x_old = []
        y_old = []
        # counts the number of runs through the loop, needed for better performing window updates
        i = 0
        # number of cars in simulation, updated in the following loop
        car_count = 0
        for data in self.g.calculation:
            self.label_time["text"] = data[1]
            if data[1] == 0:
                # save the first point of each car (timestamp equals 0)
                # 'append' is necessary to expand list to the needed size
                x_old.append(data[2])
                y_old.append(data[3])
                # if this run is the last on with timestamp equals 0, car count is constant
                car_count = len(x_old)
            else:
                # create tag to move whole shape (rect and wheels)
                car_tag = 'car'
                car_tag += str(data[0])
                # calculating vector from last point to the current point
                dx = data[2] - x_old[data[0]]
                dy = data[3] - y_old[data[0]]
                # move whole car and set this point to x_old and y_old
                self.grid.move(car_tag, dx, dy)
                x_old[data[0]] = data[2]
                y_old[data[0]] = data[3]
            if i % car_count == car_count - 1:
                # window will be updated when the last car of the timestamp is refreshed
                self.window.update()    # TODO this routine is very slow. ideas?
            i += 1
        self.label_status["text"] = "animation done!"
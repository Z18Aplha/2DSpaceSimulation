from CarFree2D import CarFree2D
from math import ceil
from Point import Point
from Obstacles2D import Obstacles2D


class God:

    # controls each car
    # interface to periphery
    # r/w from/to txt file (or sth else)
    # use (instance of CarFree2D).set_destination(x,y) to push data to each car
    # maybe: create a log file (CSV Data to copy it to excel etc to show graphs of velocity, acceleration, etc)...
    # ...for presentation
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # file_path = os.path.join(current_dir, "path.txt")
    # path_file = pygame.image.load(image_path)

    def __init__(self, dt, c_dt):
        self.cars = []  # list of each car in simulation
        self.last_timestamp = 0  # stores the last timestamp - to stop the simulation after it
        self.size = [30, 20]  # size of the canvas in m
        self.m2p_factor = 20  # factor to convert from meters to pixels
        # self.size = [0, 0]         # stores highest x and y values for matching the simulation area
        self.calculation = []  # list of polynomial for a specific period of time --> WHAT DOES THIS MEAN?
        self.dt = dt  # time between each data point in ms
        self.c_dt = c_dt  # time between each controller input (just for equidistant controller) in ms (WHY DO WE USE AN EXTRA VARIABLE AND NOT JUST EQUIDISTANT VALUES IN dt?)
        self.obstacles = []

    def file_read(self):

        ############################
        ## INITIALIZE EACH CAR #####
        ############################
        cars_txt = open("cars.txt", "r")
        # The file cars.txt consists of 10 columns, separated by commas
        #
        # [car_id,spawn_x,spawn_y,width,length,max_vel_x,max_vel_y,max_acc_x,max_acc_y,color]
        #
        # car_id:           defines the car
        # spawn_x,spawn_y:  Define the spawn point of the car. Since every car is a rectangle, this is the center of the rectangle
        # width,length:     Defines the car dimensions
        # max_vel_x,max_vel_y: Defines the maximum velocities of the car DOES THIS MAKE SENSE? I WOULD RATHER JUST DEFINE A SINGLE VALUE HERE
        # max_acc_x,max_acc_y: Defines the maximum accelerations of the car DOES THIS MAKE SENSE? I WOULD RATHER JUST DEFINE A SINGLE VALUE HERE
        # color:            Defines the car color
        for line in cars_txt:

            car_data = line.split(',')

            car_id = int(car_data[0])
            spawn_x = float(car_data[1])
            spawn_y = float(car_data[2])
            if (spawn_x < 0 or spawn_x > self.size[0] or spawn_y < 0 or spawn_y > self.size[1]):
                raise Exception('A car cannot spawn outside of canvas.')
            width = float(car_data[3])
            length = float(car_data[4])
            max_vel_x = float(car_data[5])
            max_vel_y = float(car_data[6])
            max_acc_x = float(car_data[7])
            max_acc_y = float(car_data[8])
            color = str(car_data[9])
            color = color.replace("\n", "")
            color = color.replace(" ", "")

            car = CarFree2D(car_id, spawn_x, spawn_y, width, length, max_vel_x, max_vel_y, max_acc_x, max_acc_y, color,
                            self.c_dt)
            self.cars.append(car)

        ################################
        ## ASSIGN PATH TO EACH CAR #####
        ################################
        path_txt = open("path.txt", "r")
        # The file path.txt consists of 5 columns, separated by commas. Every row consists of a single point within a path.
        #
        # [car_id,timestamp,pos_x,pos_y,destination]
        #
        # car_id:       identifies the car
        # timestamp:    at what time does the car need to be at a certain location? Unit is s
        # pos_x,pos_y:  where does the car need to be? Unit is m
        # destination:  True  = the speed of the car at the specified location is supposed to be zero
        #               False = this is a midway point
        for line in path_txt:

            path_data = line.split(',')

            car_id = int(path_data[0])
            timestamp = float(path_data[1])
            pos_x = float(path_data[2])
            pos_y = float(path_data[3])
            if (pos_x < 0 or pos_x > self.size[0] or pos_y < 0 or pos_y > self.size[1]):
                raise Exception('The path of a car cannot reach outside the canvas.')
            destination = bool(path_data[4])

            if timestamp == 0:
                raise Exception(
                    'For a timestamp of 0 you need to change the spawn point of the car, not the path.txt file.')
            elif timestamp > 0:
                if destination:
                    self.cars[car_id].set_destination(pos_x, pos_y, timestamp)
                    self.last_timestamp = max(timestamp, self.last_timestamp)
                else:
                    # car.set_waypoint(pos_x, pos_y, timestamp)
                    pass
            else:
                raise Exception('The input value for timestamp cannot be parsed correctly.')

            # CHECK IF EVERY CAR HAS AT LEAST ONE DESTINATION POINT
        for car in self.cars:
            if len(car.path.points) == 1:
                raise Exception(
                    'Every car needs at least one point (the destination) in the path file. The car with id=' + str(
                        car.id) + ' is missing.')

        #############################
        ## INITIALIZE OBSTACLES #####
        #############################
        obstacles_txt = open("obstacles.txt", "r")

        for line in obstacles_txt:

            obstacle_data = line.split(',')

            spawn_x = float(obstacle_data[0])
            spawn_y = float(obstacle_data[1])
            if spawn_x < 0 or spawn_x > self.size[0] or spawn_y < 0 or spawn_y > self.size[1]:
                raise Exception('An obstacle cannot be defined outside the canvas boundary.')
            # THIS CODE NEEDS TO BE UPDATED IN ORDER TO SUPPORT POLYGONS!
            size_x = float(obstacle_data[2])
            size_y = float(obstacle_data[3])
            color = str(obstacle_data[4])
            color = color.replace("\n", "")
            color = color.replace(" ", "")

            obstacle = Obstacles2D(spawn_x, spawn_y, size_x, size_y, color)
            self.obstacles.append(obstacle)

    def simulate(self):
        # c_dt... time between each controller input in ms

        for car in self.cars:
            car.create_spline()

        n = ceil(self.last_timestamp * 1000 / self.dt)

        for car in self.cars:
            car.update()

        for i in range(0, n + 1):
            for car in self.cars:
                self.calculation.append(car.status(i * self.dt / 1000))

    def detect_collision(self):
        pass
from CarFree2D import CarFree2D
from math import ceil
from Point import Point
from Obstacles2D import Obstacles2D
from CollisionControl import CollisionControl
from Channel import Channel
import time

class God:

    # controls each car
    # interface to periphery
    # r/w from/to txt file (or sth else)
    # use (instance of CarFree2D).set_destination(x,y) to push data to each car
    # maybe: create a log file (CSV Data to copy it to excel etc to show graphs of velocity, acceleration, etc)...
    # ...for presentation
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # path_file = pygame.image.load(image_path)

    def __init__(self, parameters):
        self.parameters = parameters
        self.cars = []  # list of each car in simulation
        self.last_timestamp = 0  # stores the last timestamp - to stop the simulation after it
        self.size = parameters["God"]["size"]  # size of the canvas in m
        self.m2p_factor = parameters["God"]["m2p_factor"]  # factor to convert from meters to pixels
        # self.size = [0, 0]         # stores highest x and y values for matching the simulation area
        self.calculation = []  # list of polynomial for a specific period of time --> WHAT DOES THIS MEAN?
        self.simulation = []
        self.controller_data = []
        self.dt = parameters["God"]["dt"]  # time between each data point in ms
        self.c_dt = parameters["God"]["c_dt"]  # time between each controller input (just for equidistant controller) in ms (WHY DO WE USE AN EXTRA VARIABLE AND NOT JUST EQUIDISTANT VALUES IN dt?)
        self.obstacles = []

    def file_read(self):
        ############################
        ## INITIALIZE EACH CAR #####
        ############################
        cars_origin = self.parameters["Cars"]
        #
        #
        # [car_id,spawn_x,spawn_y,width,length,max_vel_x,max_vel_y,max_acc_x,max_acc_y,color]
        #
        # car_id:           defines the car
        # spawn_x,spawn_y:  Define the spawn point of the car. Since every car is a rectangle, this is the center of the rectangle
        # width,length:     Defines the car dimensions
        # max_vel_x,max_vel_y: Defines the maximum velocities of the car DOES THIS MAKE SENSE? I WOULD RATHER JUST DEFINE A SINGLE VALUE HERE
        # max_acc_x,max_acc_y: Defines the maximum accelerations of the car DOES THIS MAKE SENSE? I WOULD RATHER JUST DEFINE A SINGLE VALUE HERE
        # color:            Defines the car color
        for car in cars_origin:
            car_id = int(car["index"])
            spawn_x = float(car["spawn_x"])
            spawn_y = float(car["spawn_y"])
            if (spawn_x < 0 or spawn_x > self.size[0] or spawn_y < 0 or spawn_y > self.size[1]):
                raise Exception('A car cannot spawn outside of canvas.')
            angle = float(car["angle"])
            length = float(car["length"])
            width = float(car["width"])
            max_vel = float(car["max_vel"])
            max_acc = float(car["max_acc"])
            color = str(car["color"])

            car = CarFree2D(car_id, spawn_x, spawn_y, angle, length, width, max_vel, max_acc, color,
                            self.c_dt)
            self.cars.append(car)

        ################################
        ## ASSIGN PATH TO EACH CAR #####
        ################################
        path_origin = self.parameters["Path"]
        #
        #
        # [car_id,timestamp,pos_x,pos_y,destination]
        #
        # car_id:       identifies the car
        # timestamp:    at what time does the car need to be at a certain location? Unit is s
        # pos_x,pos_y:  where does the car need to be? Unit is m
        # destination:  True  = the speed of the car at the specified location is supposed to be zero
        #               False = this is a midway point
        for path_data in path_origin:
            car_id = int(path_data["car_id"])
            pos_x = float(path_data["pos_x"])
            pos_y = float(path_data["pos_y"])
            if (pos_x < 0 or pos_x > self.size[0] or pos_y < 0 or pos_y > self.size[1]):
                raise Exception('The path of a car cannot reach outside the canvas.',car_id, pos_x, pos_y, self.size[0], self.size[1])

            #if timestamp == 0:
            #    raise Exception(
            #        'For a timestamp of 0 you need to change the spawn point of the car)
            #elif timestamp > 0:
            #if destination:
                #self.cars[car_id].set_destination(pos_x, pos_y, timestamp)
                #self.last_timestamp = max(timestamp, self.last_timestamp)
                #self.cars[car_id].set_destination(pos_x, pos_y)
            #else:
                # car.set_waypoint(pos_x, pos_y, timestamp)
                #pass
            #else:
            #    raise Exception('The input value for timestamp cannot be parsed correctly.')

            self.cars[car_id].set_waypoint(pos_x, pos_y)

            # CHECK IF EVERY CAR HAS AT LEAST ONE DESTINATION POINT
        for car in self.cars:
            if len(car.path.points) == 1:
                raise Exception(
                    'Every car needs at least one point (the destination) in the path file. The car with id=' + str(
                        car.id) + ' is missing.')

        #############################
        ## INITIALIZE OBSTACLES #####
        #############################
        obstacles_origin = self.parameters["Obstacles"]

        for obst in obstacles_origin:
            spawn_x = float(obst["corners"][0])
            spawn_y = float(obst["corners"][1])
            if spawn_x < 0 or spawn_x > self.size[0] or spawn_y < 0 or spawn_y > self.size[1]:
                raise Exception('An obstacle cannot be defined outside the canvas boundary.')
            # THIS CODE NEEDS TO BE UPDATED IN ORDER TO SUPPORT POLYGONS!
            edges = obst["corners"]
            color = obst["color"]

            obstacle = Obstacles2D(spawn_x, spawn_y, edges, color)
            self.obstacles.append(obstacle)

        # Adding outer boundary as obstacles
        self.obstacles.append(Obstacles2D(0, 0, [0, 0, self.size[0], 0, self.size[0], -1, 0, -1], ''))
        self.obstacles.append(Obstacles2D(-1, self.size[1], [-1, self.size[1], 0, self.size[1], 0, 0, -1, 0], ''))
        self.obstacles.append(Obstacles2D(0, self.size[1]+1, [0, self.size[1]+1, self.size[0], self.size[1]+1,
                                                              self.size[0], self.size[1], 0, self.size[1]], ''))
        self.obstacles.append(Obstacles2D(self.size[0], self.size[1], [self.size[0], self.size[1], self.size[0]+1,
                                                                       self.size[1], self.size[0]+1, 0, self.size[0],
                                                                       0], ''))

    def simulate_backup(self):
        # c_dt... time between each controller input in ms
        for car in self.cars:
            car.create_spline()

        n = ceil(self.last_timestamp * 1000 / self.dt)

        for car in self.cars:
            car.update()

        for i in range(0, n + 1):
            for car in self.cars:
                self.calculation.append(car.status(i * self.dt / 1000))

        coll = CollisionControl(self)
        coll.check_for_collision()

        c = Channel(self.parameters)
        for s in self.calculation:
            s_new = c.send(s[:])
            self.simulation.append(s_new)

        for dat in self.simulation:
            data = dat[:]
            data[1] = ceil(data[1] / (self.c_dt / 1000)) * (self.c_dt / 1000)
            if not data[-1]:
                if len(self.controller_data) <= len(self.cars):
                    for i in range(2, len(data) - 2):
                        data[i] = 0
                else:
                    for obj in reversed(self.controller_data):
                        if data[0] == obj[0]:
                            for i in range(2, len(obj)-2):
                                data[i] = obj[i]
            del data[-1]
            self.controller_data.append(data)

    def simulate(self):
        # c_dt... time between each controller input in ms
        for car in self.cars:
            car.create_spline()

        for car in self.cars:
            car.update2()


        n = 0
        cars = len(self.cars)
        lists_ended = 0
        while lists_ended < cars:
            lists_ended = 0
            for car in self.cars:
                try:
                    entry = [car.id, car.position[n][0], car.position[n][1], car.position[n][2]]
                    self.calculation.append(entry)
                except IndexError:
                    lists_ended += 1
                    pass
            n+=1
        pass


        #coll = CollisionControl(self)
        #coll.check_for_collision()

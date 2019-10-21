from newCarFree2D import CarFree2D
from math import ceil
from Obstacles2D import Obstacles2D
from CollisionControl import CollisionControl
from EventQueue import EventQueue
from Event import Event
import Lib as lib
from scipy import signal
import control


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
        # self.size = [0, 0]         # stores highest x and y values for matching the simulation area
        self.calculation = []  # list of polynomial for a specific period of time --> WHAT DOES THIS MEAN?
        self.simulation = []
        self.controller_data = []
        dt = parameters["God"]["dt"]  # time between each data point in ms
        lib.set_dt(dt)
        self.ct = parameters["God"]["ct"]  # time between each controller input (just for equidistant controller) in ms (WHY DO WE USE AN EXTRA VARIABLE AND NOT JUST EQUIDISTANT VALUES IN dt?)
        lib.set_ct(self.ct)
        lib.set_fps(parameters["SpaceFree2D"]["fps"])
        self.obstacles = []
        self.colldet = self.parameters["CollisionControl"]["activated"]
        self.collisions = [10000, 10000, 10000]
        self.latency = parameters["God"]["latency"]
        # INSERT IN LIBRARY
        lib.set_coll_det_freq(parameters["CollisionControl"]["collision_detection_frequency"])
        eq = EventQueue(self)
        lib.set_eventqueue(eq)
        lib.set_latency(self.latency)
        lib.set_k_d(parameters["God"]["k_d"])
        lib.set_k_p(parameters["God"]["k_p"])
        lib.set_pt(parameters["God"]["pt"])
        self.eventlist_debug = []
        self.make_statespace()


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

            car = CarFree2D(car_id, spawn_x, spawn_y, length, width, angle, max_vel, max_acc, color,
                            self.ct)
            self.cars.append(car)
            lib.carList.append(car)

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

            try:
                self.cars[car_id].set_waypoint(pos_x, pos_y)
            except IndexError:
                print("No car with matching ID found")
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
        # Bottom
        self.obstacles.append(Obstacles2D(0.1, 0.1, [0.1, 0.1, self.size[0], 0.1, self.size[0], -1, 0.1, -1], 'white'))
        # Left
        self.obstacles.append(Obstacles2D(-1, self.size[1], [-1, self.size[1], 0.1, self.size[1], 0.1, 0.1, -1, 0.1], 'white'))
        # Top
        self.obstacles.append(Obstacles2D(0.1, self.size[1]+1, [0.1, self.size[1]+1, self.size[0]-0.1, self.size[1]+1,
                                                              self.size[0]-0.1, self.size[1]-0.1, 0.1, self.size[1]-0.1], 'white'))
        # Right
        self.obstacles.append(Obstacles2D(self.size[0]-0.1, self.size[1]-0.1, [self.size[0]-0.1, self.size[1], self.size[0]+1,
                                                                       self.size[1], self.size[0]+1, 0, self.size[0]-0.1,
                                                                       0], 'white'))

        lib.set_collision(CollisionControl(self))
        lib.set_carcount(len(self.cars))

        # Make DC-motor
    def make_statespace(self):
        Jm = self.parameters["DC-Motor"]["Jm"]
        Bm = self.parameters["DC-Motor"]["Bm"]
        Kme = self.parameters["DC-Motor"]["Kme"]
        Kmt = self.parameters["DC-Motor"]["Kmt"]
        Rm = self.parameters["DC-Motor"]["Rm"]
        Lm = self.parameters["DC-Motor"]["Lm"]

        Kdm = self.parameters["DC-Motor"]["Kdm"]
        Kpm = self.parameters["DC-Motor"]["Kpm"]
        Kim = self.parameters["DC-Motor"]["Kim"]
        Nm = self.parameters["DC-Motor"]["Nm"]

        Ts = 0.005
        DC = control.TransferFunction([0, Kmt], [Jm * Lm, Bm * Lm + Jm * Rm, Bm * Rm + Kme * Kmt])
        PIDm = control.TransferFunction([Kpm + Kdm * Nm, Kpm * Nm + Kim, Kim * Nm], [1, Nm, 0])

        II = control.TransferFunction([1], [1, 0, 0])

        AGV = II * control.feedback(DC*PIDm, sign=-1)

        #AGV = II * (PIDm * DC) / (1 + PIDm * DC)

        AGVz = control.sample_system(AGV, Ts, method='zoh')

        SS = control.tf2ss(AGVz)

        lib.set_statespace(SS)

    # OLD - not used anymore
    def simulate_backup(self):
        # c_dt... time between each controller input in ms
        for car in self.cars:
            car.create_spline()

        n = ceil(self.last_timestamp * 1000 / lib.dt)

        for car in self.cars:
            car.update()

        coll = CollisionControl(self)

        for i in range(0, n + 1):
            for car in self.cars:
                self.calculation.append(car.status(i * lib.dt))
            if coll.check_for_collision() is False:
                print("Collision occourred . . . Aborting")
                break

        for dat in self.simulation:
            data = dat[:]
            data[1] = ceil(data[1] / (self.ct / 1000)) * (self.ct / 1000)
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

        # Spline-Creation
        for car in self.cars:
            event = Event(-1, car, (car, ), lambda: lib.eventqueue.create_spline)
            lib.eventqueue.add_event(event)

        # executing the entries of the eventqueue
        while lib.eventqueue.has_elements():
            event = lib.eventqueue.events.pop(0)

            #######
            # only for debugging
            try:
                self.eventlist_debug.append(event.time)
            except AttributeError:
                self.eventlist_debug.append([event.function, event.object, event.parameters])
            ########
            pass
            lib.eventqueue.exe(event.function(), event.parameters)

        self.last_timestamp = lib.data[-1][0]

        # Collision Detection
        if self.colldet:
            coll = CollisionControl(self)
            coll.check_for_collision()

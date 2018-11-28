#import os
# import pygame
from math import tan, radians, degrees, copysign, sqrt
import time


class Point:
    def __init__(self, x: float, y: float, t: float, destination: bool):
        self.x = x
        self.y = y
        self.time = t
        self.is_destination = destination


class Path:
    def __init__(self, position, t):
        p = Point(position[0], position[1], t, True)
        self.points = []
        self.add(p)

    def add(self, p: Point):
        self.points.append(p)


class Controller:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # each car has its own controller
    # class with path planning and path following algorithms

    def __init__(self, p: Path, max_acceleration, max_velocity):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.path = p
        self.control = [0.0, 0.0, p.points[0].time]
        self.controls = []
        self.controls.append(self.control)

    def check_possibility(self, last: Point, new: Point):  # recently: just for linear controlling
        distance = [abs(last.x - new.x), abs(last.y - new.y)]

        if last.time < time.time():
            t_between = new.time - last.time
        else:
            t_between = new.time - time.time()

        distance_acceleration = [self.max_velocity[0] * self.max_velocity[0] / self.max_acceleration[0],
                                 self.max_velocity[1] * self.max_velocity[1] / self.max_acceleration[1]]
        if distance[0] > distance_acceleration[0] or distance[1] > distance_acceleration[1]:
            t_needed = max((2 * self.max_velocity[0] / self.max_acceleration[0] + (
                        distance[0] - distance_acceleration[0]) / self.max_velocity[0]), (
                                       2 * self.max_velocity[1] / self.max_acceleration[1] + (
                                           distance[1] - distance_acceleration[1]) / self.max_velocity[1]))
            possible = t_between > t_needed
        else:
            possible = True

        return possible

    def create_path(self, last: Point, destination: Point):
        # first step: linear

        last_index = len(self.path.points) - 2
        t = destination.time - last.time
        distance = [abs(last.x - destination.x), abs(last.y - destination.y)]

        vx = -0.5*(sqrt((-self.max_acceleration[0])*(-self.max_acceleration[0]*t*t + 4*distance[0]))-self.max_acceleration[0]*t)
        vy = -0.5*(sqrt((-self.max_acceleration[1])*(-self.max_acceleration[1]*t*t + 4*distance[1]))-self.max_acceleration[1]*t)

        vx_max = max(-self.max_velocity[0], min(vx, self.max_velocity[0]))
        vy_max = max(-self.max_velocity[1], min(vy, self.max_velocity[1]))

        tx_to_max = vx_max / self.max_acceleration[0]
        ty_to_max = vy_max / self.max_acceleration[1]
        t = 0  # for making the next lines more comfortable

        # STARTING POINT
        if tx_to_max > ty_to_max:
            # x component is limiting --> ax = maximum, ay needs to be adapted
            # ty_to_max must be equal to tx_to_max!
            t = tx_to_max
            ax = self.max_acceleration[0]
            ay = vy_max / t
        else:
            # y component limits
            t = ty_to_max
            ax = vx_max / t
            ay = self.max_acceleration[1]

        control = [ax, ay, last.time]
        self.controls.insert(last_index + 1, control)

        # ACCELERATION TO ZERO POINT
        control = [0.0, 0.0, time.time()+t]
        self.controls.insert(last_index + 2, control)

        # BRAKING POINT
        control = [-ax, -ay, destination.time - t]
        self.controls.insert(last_index + 3, control)

        # STOP POINT
        control = [0.0, 0.0, destination.time]
        self.controls.insert(last_index + 4, control)


class God:
    # TODO design omni-potent class
    # controls each car
    # interface to periphery
    # r/w from/to txt file (or sth else)
    # use (instance of CarFree2D).set_destination(x,y) to push data to each car
    # maybe: create a log file (CSV Data to copy it to excel etc to show graphs of velocity, acceleration, etc)...
    # ...for presentation
    #current_dir = os.path.dirname(os.path.abspath(__file__))
    #file_path = os.path.join(current_dir, "path.txt")
    #path_file = pygame.image.load(image_path)
    fileToBeRead = open("path.txt", "r")
    xCoord = 0.0
    yCoord = 0.0
    timestamp = 0 #milliseconds
    destination = False
    for line in fileToBeRead:
        xCoord = line.split(',')[0]
        yCoord = line.split(',')[1]
        timestamp = line.split(',')[2]
        destination = line.split(',')[3]
        #print(xCoord,yCoord,timestamp,destination)
        p = Point(xCoord,yCoord,timestamp,destination)
        
class SpaceSimulation2D:
    def __init__(self, y, x):
        self.high = y
        self.width = x
        self.px_high = 1920
        self.px_width = 1080
        self.high_ratio = self.px_high / self.high
        self.width_ratio = self.px_width / self.width
        # TODO complete instructor of SpaceSimulation2D
        # TODO design Space for Simulation class
        # scalable - width/pixels = metres/Pixel
        # later: obstacles


class CarFree2D:
    def __init__(self, x: float, y: float):
        # PROPERTIES
        self.position = [x, y]  # in m - later: instantiation of 2d space with dates in metres
        self.length = 400  # [length] = cm
        self.width = 200  # [width] = cm
        self.timestamp = time.time()  # current time in UNIX
        # VELOCITY (m/s)
        self.velocity = [0.0, 0.0]
        self.max_velocity = [50.0, 10.0]  # [vx, vy]
        # ACCELERATION (DE-) (m/s^2)
        self.acceleration = [0.0, 0.0]  # [ax, ay]
        self.max_acceleration = [30.0, 15.0]  # [ax, ay]
        # PATH
        self.path = Path(self.position, self.timestamp)
        self.controller = Controller(self.path, self.max_acceleration, self.max_velocity)

    # GETTER
    def get_position(self):
        return self.position

    def get_speed(self, absolute):  # absolute is boolean - true returns absolute value, false vectorial speed
        if absolute:
            return sqrt(self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1])
        else:
            return self.velocity

    def get_acceleration(self):
        return self.acceleration

    def status(self):
        return [self.position[0], self.position[1], self.velocity[0],
                self.velocity[1], self.acceleration[0], self.acceleration[1], time.time()]  # experimental, maybe further advantages?

    # SETTER
    def set_acceleration(self, ax: float, ay: float):
        # CHECK RANGE
        ax = max(-self.max_acceleration[0], min(ax, self.max_acceleration[0]))
        ay = max(-self.max_acceleration[1], min(ay, self.max_acceleration[1]))
        self.acceleration = [ax, ay]

    def set_velocity(self, vx: float, vy: float):
        # CHECK RANGE
        vx = max(-self.max_velocity[0], min(vx, self.max_velocity[0]))
        vy = max(-self.max_velocity[1], min(vy, self.max_velocity[1]))
        self.velocity = [vx, vy]

    def set_destination(self, x, y, t):
        p = Point(x, y, t, True)
        if self.controller.check_possibility(self.path.points[-1], p):
            self.path.add(p)
            if self.path.points[-2].time > time.time():
                self.controller.create_path(self.path.points[-2], p)
            else:
                p_now = Point(self.position[0], self.position[1], time.time(), True)
                self.controller.create_path(p_now, p)
        else:
            print("The point (" + p.x + "|" + p.y + ") is to far away. Skipped.")

    # SIMULATION
    def update(self):

        dt = (time.time() - self.timestamp)  # [dt] = seconds
        self.timestamp = time.time()

        # ACCELERATION
        found = False
        for control in self.controller.controls:
            if not found and control[2] > self.timestamp: #and not (control[0] == self.acceleration[0] and control[1] == self.acceleration[1]):
                # found a new acceleration input
                found = True
            elif not found:
                ax = control[0]
                ay = control[1]

        self.set_acceleration(ax, ay)

        # VELOCITY
        vx = self.velocity[0] + dt * self.acceleration[0]
        vy = self.velocity[1] + dt * self.acceleration[1]
        self.set_velocity(vx, vy)

        # POSITION
        x = self.position[0] + dt * self.velocity[0]
        y = self.position[1] + dt * self.velocity[1]
        self.position = [x, y]


# TESTING ROUTINE


car1 = CarFree2D(0, 0)
car1.set_destination(10, 10, time.time() + 10)
i = 0
while 1 < 2:
    i = i + 1
    car1.update()
    if (i % 10000) == 0:  # skips showing some update routines - better overview
        #print(car1.status())
        print(time.time(), car1.position[0], car1.position[1], car1.acceleration[0], car1.acceleration[1])

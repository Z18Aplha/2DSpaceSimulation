from CarFree2D import CarFree2D
from math import ceil


class God:
    def __init__(self, dt, c_dt):
        self.cars = []  # list of each car in simulation
        self.last_timestamp = 0  # stores the last timestamp - to stop the simulation after it
        self.size = [0, 0]  # stores highest x and y values for matching the simulation area
        self.calculation = []  # list if polynomial for a specific period of time
        self.dt = dt    # time between each data point
        self.c_dt = c_dt    # time between each controller input (just for equidistant controller)

    # controls each car
    # interface to periphery
    # r/w from/to txt file (or sth else)
    # use (instance of CarFree2D).set_destination(x,y) to push data to each car
    # maybe: create a log file (CSV Data to copy it to excel etc to show graphs of velocity, acceleration, etc)...
    # ...for presentation
    # current_dir = os.path.dirname(os.path.abspath(__file__))
    # file_path = os.path.join(current_dir, "path.txt")
    # path_file = pygame.image.load(image_path)

    def file_read(self):
        # INITIALIZE EACH CAR
        cars_txt = open("cars.txt", "r")
        for line in cars_txt:
            car_id = int(line.split(',')[0])
            spawn_x = float(line.split(',')[1])
            self.size[0] = max((self.size[0], spawn_x))
            spawn_y = float(line.split(',')[2])
            self.size[1] = max((self.size[1], spawn_y))
            size_x = float(line.split(',')[3])
            size_y = float(line.split(',')[4])
            max_vel_x = float(line.split(',')[5])
            max_vel_y = float(line.split(',')[6])
            max_acc_x = float(line.split(',')[7])
            max_acc_y = float(line.split(',')[8])
            color = str(line.split(',')[9])
            color = color.replace("\n", "")
            color = color.replace(" ", "")

            car = CarFree2D(car_id, spawn_x, spawn_y, size_x, size_y, max_vel_x, max_vel_y, max_acc_x, max_acc_y, color, self.c_dt)
            self.cars.append(car)

        # READ PATH OF EACH CAR
        path_txt = open("path.txt", "r")
        x = 0.0
        y = 0.0
        timestamp = 0  # seconds
        destination = True
        for line in path_txt:
            car_id = int(line.split(',')[0])
            timestamp = float(line.split(',')[1])
            x = float(line.split(',')[2])
            self.size[0] = max((self.size[0], x))
            y = float(line.split(',')[3])
            self.size[1] = max((self.size[1], y))
            destination = bool(line.split(',')[4])
            # print(xCoord,yCoord,timestamp,destination)
            # self.cars.append())
            # car1.set_destination(10, 10, time.time() + 10)
            if timestamp == 0:
                # error
                pass
            elif timestamp > 0:
                for car in self.cars:
                    if car.id == car_id:
                        if destination:
                            car.set_destination(x, y, timestamp)
                            self.last_timestamp = max(timestamp, self.last_timestamp)
                        else:
                            # car.set_waypoint(x, y, timestamp)
                            pass
            else:
                # error_output
                pass

    def calculate_linear_event(self):

        # LAST TIMESTAMP
        # n is #of of outputs
        n = ceil(self.last_timestamp * 1000 / self.dt)

        for car in self.cars:
            car.update()

        for i in range(0, n + 1):
            for car in self.cars:
                self.calculation.append(car.status(i * self.dt / 1000))

    def calculate_spline_equidistant(self):
        # c_dt... time between each controller input in ms

        for car in self.cars:
            car.create_spline()

        n = ceil(self.last_timestamp * 1000 / self.dt)

        for car in self.cars:
            car.update()

        for i in range(0, n + 1):
            for car in self.cars:
                self.calculation.append(car.status(i * self.dt / 1000))

from CarFree2D import CarFree2D
from math import ceil


class God:
    def __init__(self):
        self.cars = []
        self.last_timestamp = 0
        self.calculation = []

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
        fileToBeRead = open("path.txt", "r")
        x = 0.0
        y = 0.0
        timestamp = 0  # seconds
        destination = True
        for line in fileToBeRead:
            car_id = int(line.split(',')[0])
            timestamp = float(line.split(',')[1])
            x = float(line.split(',')[2])
            y = float(line.split(',')[3])
            destination = bool(line.split(',')[4])
            # print(xCoord,yCoord,timestamp,destination)
            # self.cars.append())
            # car1.set_destination(10, 10, time.time() + 10)
            if timestamp == 0:
                self.cars.append(CarFree2D(x, y, car_id))
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

    def calculate(self, dt):

        # LAST TIMESTAMP
        # dt in ms, timestamp in s
        # n is #of of outputs
        n = ceil(self.last_timestamp * 1000 / dt)

        for car in self.cars:
            car.update()

        for i in range(0,n+1):
            for car in self.cars:
                self.calculation.append(car.status(i*dt/1000))



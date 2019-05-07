from CarFree2D import CarFree2D
from random import *


class Channel:

    def __init__(self, pa):
        self.latency = pa["Channel"]["latency"]
        self.min_latency = pa["Channel"]["min_latency"]
        self.max_latency = pa["Channel"]["max_latency"]
        self.error_probability = pa["Channel"]["error_probability"]

    def send(self, elem):
        elem[1] += 2 * randrange(self.min_latency, self.max_latency) / 1000
        if random() < self.error_probability:
            elem.append(False)
        else:
            elem.append(True)
        return elem

    def request(self, car: CarFree2D, time):
        data = car.status(time)
        answer = [(random() > self.error_probability), data, 2*self.latency]
        return answer

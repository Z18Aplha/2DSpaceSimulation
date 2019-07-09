#from CarFree2D import CarFree2D
from Obstacles2D import Obstacles2D
import Lib as lib

class Event:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # each car has its own controller
    # class with path planning and path following algorithms


    def __init__(self, t, obj, parameters, function):
        self.time = t
        self.object = obj
        self.function = function
        self.parameters = parameters


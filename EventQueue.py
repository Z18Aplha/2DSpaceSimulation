from CarFree2D import CarFree2D
from Event import Event
import God


class EventQueue:

    def __init__(self, god):
        self.events = []
        self.last_index = 0
        self.god = god

    def add_event(self, event: Event):
        # sorting, replacing, controlling
        self.events.append(event)

    def execute(self, x, event_index):
        try:
            #self.events[event_index].function(self.events[event_index].object)
            self.events[event_index].function()
            self.last_index = event_index
            done = True
        except ValueError:
            done = False

        return done

    def exe(self, x, y):
        x(*y)

    ###INCLUDED FUNCTIONS

    def create_spline(self, car):
        car = self.god.cars[car.id]
        car.create_spline()

    def fn(self, x):
        print(x*10)
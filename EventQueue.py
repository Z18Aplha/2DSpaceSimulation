from CarFree2D import CarFree2D
from Event import Event
import Lib as lib


class EventQueue:

    def __init__(self, god):
        self.events = []
        self.last_index = 0
        self.god = god
        self.last_get_data = 0
        self.get_data_counter = 0
        self.add_event(Event(0, None, (0,), lambda: lib.eventqueue.get_data))

    def add_event_old(self, event: Event):
        # sorting, replacing, controlling
        self.events.append(event)

    def add_event(self, event: Event):
        notinserted = True
        for e in self.events[::-1]:
            if event.time >= e.time:
                index = self.events.index(e)
                if (event.time == e.time) & (event.object == e.object):
                    self.events.remove(e)
                    self.events.insert(index, event)
                    notinserted = False
                    break
                else:
                    if self.events[index-1].time != event.time:
                        self.events.insert(index+1, event)
                        notinserted = False
                        break
        if notinserted:
            self.events.insert(0, event)

        # Add get_data
        difference = event.time - (self.last_get_data + lib.dt)
        to_add = int(difference / lib.dt)
        for i in range(to_add):
            self.last_get_data += lib.dt
            data_event = (self.last_get_data, None, 0, lambda: lib.eventqueue.get_data())
        # TODO every dt: get_data

    # def execute(self, x, event_index):
    #     try:
    #         #self.events[event_index].function(self.events[event_index].object)
    #         self.events[event_index].function()
    #         self.last_index = event_index
    #         done = True
    #     except ValueError:
    #         done = False
    #
    #     return done

    def exe(self, x, y):
        x(*y)

    ###INCLUDED FUNCTIONS

    def create_spline(self, car):
        car = self.god.cars[car.id]
        car.create_spline()

    def car_control(self, t, car: CarFree2D, acc, direction, stop):
        car.control(t, acc, direction, stop)

    def get_data(self, t):
        for car in lib.carList:
            a = car.get_data(t)
            print(a)
            lib.data.append(a)

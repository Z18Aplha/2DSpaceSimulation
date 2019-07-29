from CarFree2D import CarFree2D
from Event import Event
import Lib as lib


class EventQueue:

    def __init__(self, god):
        self.events = []
        self.last_index = 0
        self.god = god
        self.last_get_data = 0
        self.last_coll_control = 0
        self.add_event(Event(0, None, (0,), lambda: lib.eventqueue.check_for_collision()))
        self.add_event(Event(0, None, (0,), lambda: lib.eventqueue.get_data))

    def add_event_old(self, event: Event):
        # sorting, replacing, controlling
        self.events.append(event)

    def add_event(self, event: Event):
        not_inserted = True
        for e in self.events[::-1]:
            if event.time >= e.time:
                index = self.events.index(e)
                if (event.time == e.time) & (event.object == e.object):
                    self.events.remove(e)
                    self.events.insert(index, event)
                    not_inserted = False
                    break
                else:
                    if self.events[index-1].time != event.time:
                        self.events.insert(index+1, event)
                        not_inserted = False
                        break
        if not_inserted:
            self.events.insert(0, event)

        # Add get_data
        difference = event.time - self.last_get_data
        to_add = int(difference / (lib.dt/1000))
        for i in range(to_add):
            self.last_get_data = round(self.last_get_data + (lib.dt/1000), 7)
            data_event = Event(self.last_get_data, None, (self.last_get_data,), lambda: lib.eventqueue.get_data)
            lib.eventqueue.add_event(data_event)

        # Add check_for_collision
        difference = event.time - self.last_coll_control
        to_add = int(difference / lib.coll_det_freq)
        for i in range(to_add):
            self.last_coll_control = round(self.last_coll_control + lib.coll_det_freq, 7)
            coll_event = Event(self.last_coll_control, None, (self.last_coll_control,), lambda: lib.eventqueue.check_for_collision)
            lib.eventqueue.add_event(coll_event)


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

    # INCLUDED FUNCTIONS
    # linking functions to "real" function

    def create_spline(self, car):
        car = self.god.cars[car.id]
        car.create_spline()

    def car_control(self, t, car: CarFree2D, acc, direction, stop):
        car.control(t, acc, direction, stop)

    def get_data(self, t):
        for car in lib.carList:
            lib.data.append(car.get_data(t))

    def check_for_collision(self, t):
        lib.collision.predict_collision(t)
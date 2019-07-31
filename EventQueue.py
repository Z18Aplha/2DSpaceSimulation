from CarFree2D import CarFree2D
from Event import Event
import Lib as lib
import copy


class EventQueue:

    def __init__(self, god):
        self.events = []
        self.last_index = 0
        self.god = god
        self.last_get_data = 0
        self.last_coll_control = 0
        self.add_event(Event(0, None, (0,), lambda: lib.eventqueue.check_for_collision))
        self.add_event(Event(0, None, (0,), lambda: lib.eventqueue.get_data))

    def add_event(self, event: Event):
        not_inserted = True

        # iterates the current events starting with the last one
        for e in self.events[::-1]:
            if event.time >= e.time:
                index = self.events.index(e)
                # if an event with identical time, obj and function is added, the old one
                # gets deleted
                if (event.time == e.time) & (event.object == e.object) & (event.function == e.function):
                    self.events.remove(e)
                    self.events.insert(index, event)
                    not_inserted = False
                    break
                else:
                    if self.events[index-1].time != event.time:
                        self.events.insert(index+1, event)
                        not_inserted = False
                        break
        # if the event is not inserted yet, it has to be inserted as the first
        if not_inserted:
            self.events.insert(0, event)

        # Add get_data
        difference = event.time - self.last_get_data
        to_add = int(difference / (lib.dt/1000))
        for i in range(to_add):
            self.last_get_data = round(self.last_get_data + (lib.dt/1000), 7)
            data_event = Event(self.last_get_data, None, (self.last_get_data,), lambda: lib.eventqueue.get_data)
            lib.eventqueue.add_event(data_event)

        # Add check_for_collision - deactivated at the moment for runtime reasons
        difference = event.time - self.last_coll_control
        to_add = int(difference / lib.coll_det_freq)
        for i in range(to_add):
            self.last_coll_control = round(self.last_coll_control + lib.coll_det_freq, 7)
            coll_event = Event(self.last_coll_control, None, (self.last_coll_control,), lambda: lib.eventqueue.check_for_collision)
            #lib.eventqueue.add_event(coll_event)

    def exe(self, x, y):
        x(*y)

    # INCLUDED FUNCTIONS
    # linking functions to "real" function

    # creates the paths of the cars
    def create_spline(self, car):
        car = self.god.cars[car.id]
        car.create_spline()
        # if the latency differs from 0 a copy of the car is created which has no
        # latency and is displayed transparently
        if not lib.latency == 0:
            #car.last_time_control = lib.latency / 1000
            car_copy = copy.deepcopy(car)
            car_copy.ghost = True
            car_copy.id = str(car_copy.id) + ' Ghost'

            # command to generate event queue entries
            car_copy.make_controls()

            self.god.cars.append(car_copy)
            lib.carList.append(car_copy)

    # gives the cars their acc and dir values
    def car_control(self, t, car: CarFree2D, acc, direction, stop):
        car.control(t, acc, direction, stop)
        #print(t, acc, direction, stop)

    # appends the current state to the library, its entries are needed to
    # display the car in the animation
    def get_data(self, t):
        for car in lib.carList:
            lib.data.append(car.get_data(t))

    def check_for_collision(self, t):
        lib.collision.predict_collision(t)

    # the stored command gets applied
    def apply_control(self, func, parameters):
        self.exe(func(), parameters)

    # a command is stored in the eventqueue,
    def store_command(self, t, obj, func, parameters):
        if obj.ghost:
            latency = 0
        else:
            latency = lib.latency/1000

        # Adding the latency
        par = list(parameters)
        par[0] += latency
        parameters = tuple(par)
        t += latency

        # storing command in the eventqueue, the command is later run by apply_control
        self.add_event(Event(t, obj, (func, parameters), lambda: lib.eventqueue.apply_control))

    def has_elements(self):
        return not len(self.events) == 0

from CarFree2D import CarFree2D
from Event import Event

class EventQueue:

    def __init__(self):
        self.events = []
        self.last_index = 0

    def add_event(self, event: Event):
        # sorting, replacing, controlling
        pass

    def execute(self, event_index):
        try:
            self.events[event_index].function()
            self.last_index = event_index
            done = True
        except:
            done = False

        return done

    ###INCLUDED FUNCTIONS

    def fn(self, x):
        return x*10
class Obstacles2D:
    def __init__(self, spawn_x, spawn_y, size_x, size_y, color):
        # PROPERTIES
        self.color = color
        self.spawn = [spawn_x, spawn_y]
        self.position = []  # in m - later: instantiation of 2d space with dates in metres
        self.length = size_x  # [length] = m
        self.width = size_y  # [width] = m


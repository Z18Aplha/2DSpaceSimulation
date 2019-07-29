class Obstacles2D:
    def __init__(self, spawn_x, spawn_y, corners: list, color):
        # PROPERTIES
        self.id = 99    # TODO: obstacle id
        self.color = color
        self.spawn = [spawn_x, spawn_y]
        self.position = []  # in m - later: instantiation of 2d space with dates in metres
        self.edges = corners


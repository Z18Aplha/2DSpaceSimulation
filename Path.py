from Point import Point


class Path:
    def __init__(self, position):
        p = Point(position[0], position[1], 0, True)
        self.points = []
        self.add(p)
        # TODO better boundary conditions (hermite)
        for i in range(1, 2):                                                  # worse way to force f'(start) = 0
            self.add(Point(position[0], position[1], i/10000, True))

    def add(self, p: Point):
        self.points.append(p)

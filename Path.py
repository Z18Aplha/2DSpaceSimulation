from Point import Point


class Path:
    def __init__(self, position):
        p = Point(position[0], position[1], 0, True)
        self.points = []
        self.add(p)

    def add(self, p: Point):
        self.points.append(p)
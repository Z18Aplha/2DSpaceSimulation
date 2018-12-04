class Path:
    def __init__(self, position, t):
        p = Point(position[0], position[1], t, True)
        self.points = []
        self.add(p)

    def add(self, p: Point):
        self.points.append(p)
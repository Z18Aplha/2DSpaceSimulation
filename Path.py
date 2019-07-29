from Point import Point


class Path:
    def __init__(self, position):
        p = Point(position[0], position[1])
        self.points = []  # list of Point objects
        self.add(p)  # add the spawn point
        # TODO better boundary conditions (hermite) --> WAS HEISST DAS?

        # bad approach to force f'(start) = 0 --> IS THIS THE VELOCITY? WE NEED TO MAKE THIS CLEANER
        # for ii in range(1, 2):
        #    self.add(Point(position[0], position[1], ii/10000, True))

    def add(self, p: Point):
        self.points.append(p)

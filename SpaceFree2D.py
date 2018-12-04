class SpaceSimulation2D:
    def __init__(self, y, x):
        self.high = y
        self.width = x
        self.px_high = 1920
        self.px_width = 1080
        self.high_ratio = self.px_high / self.high
        self.width_ratio = self.px_width / self.width
        # TODO complete instructor of SpaceSimulation2D
        # TODO design Space for Simulation class
        # scalable - width/pixels = metres/Pixel
        # later: obstacles
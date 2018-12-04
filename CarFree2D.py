class CarFree2D:
    def __init__(self, x: float, y: float):
        # PROPERTIES
        self.position = [x, y]  # in m - later: instantiation of 2d space with dates in metres
        self.length = 400  # [length] = cm
        self.width = 200  # [width] = cm
        self.timestamp = time.time()  # current time in UNIX
        # VELOCITY (m/s)
        self.velocity = [0.0, 0.0]
        self.max_velocity = [50.0, 10.0]  # [vx, vy]
        # ACCELERATION (DE-) (m/s^2)
        self.acceleration = [0.0, 0.0]  # [ax, ay]
        self.max_acceleration = [30.0, 15.0]  # [ax, ay]
        # PATH
        self.path = Path(self.position, self.timestamp)
        self.controller = Controller(self.path, self.max_acceleration, self.max_velocity)

    # GETTER
    def get_position(self):
        return self.position

    def get_speed(self, absolute):  # absolute is boolean - true returns absolute value, false vectorial speed
        if absolute:
            return sqrt(self.velocity[0] * self.velocity[0] + self.velocity[1] * self.velocity[1])
        else:
            return self.velocity

    def get_acceleration(self):
        return self.acceleration

    def status(self):
        return [self.position[0], self.position[1], self.velocity[0],
                self.velocity[1], self.acceleration[0], self.acceleration[1], time.time()]  # experimental, maybe further advantages?

    # SETTER
    def set_acceleration(self, ax: float, ay: float):
        # CHECK RANGE
        ax = max(-self.max_acceleration[0], min(ax, self.max_acceleration[0]))
        ay = max(-self.max_acceleration[1], min(ay, self.max_acceleration[1]))
        self.acceleration = [ax, ay]

    def set_velocity(self, vx: float, vy: float):
        # CHECK RANGE
        vx = max(-self.max_velocity[0], min(vx, self.max_velocity[0]))
        vy = max(-self.max_velocity[1], min(vy, self.max_velocity[1]))
        self.velocity = [vx, vy]

    def set_destination(self, x, y, t):
        p = Point(x, y, t, True)
        if self.controller.check_possibility(self.path.points[-1], p):
            self.path.add(p)
            if self.path.points[-2].time > time.time():
                self.controller.create_path(self.path.points[-2], p)
            else:
                p_now = Point(self.position[0], self.position[1], time.time(), True)
                self.controller.create_path(p_now, p)
        else:
            print("The point (" + p.x + "|" + p.y + ") is to far away. Skipped.")

    # SIMULATION
    def update(self):

        dt = (time.time() - self.timestamp)  # [dt] = seconds
        self.timestamp = time.time()

        # ACCELERATION
        found = False
        for control in self.controller.controls:
            if not found and control[2] > self.timestamp: #and not (control[0] == self.acceleration[0] and control[1] == self.acceleration[1]):
                # found a new acceleration input
                found = True
            elif not found:
                ax = control[0]
                ay = control[1]

        self.set_acceleration(ax, ay)

        # VELOCITY
        vx = self.velocity[0] + dt * self.acceleration[0]
        vy = self.velocity[1] + dt * self.acceleration[1]
        self.set_velocity(vx, vy)

        # POSITION
        x = self.position[0] + dt * self.velocity[0]
        y = self.position[1] + dt * self.velocity[1]
        self.position = [x, y]
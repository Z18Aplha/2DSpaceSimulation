class Controller:
    # assumption: acceleration is a instant value of the car --> using max_acceleration and max_deceleration
    # each car has its own controller
    # class with path planning and path following algorithms

    def __init__(self, p: Path, max_acceleration, max_velocity):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.path = p
        self.control = [0.0, 0.0, p.points[0].time]
        self.controls = []
        self.controls.append(self.control)

    def check_possibility(self, last: Point, new: Point):  # recently: just for linear controlling
        distance = [abs(last.x - new.x), abs(last.y - new.y)]

        if last.time < time.time():
            t_between = new.time - last.time
        else:
            t_between = new.time - time.time()

        distance_acceleration = [self.max_velocity[0] * self.max_velocity[0] / self.max_acceleration[0],
                                 self.max_velocity[1] * self.max_velocity[1] / self.max_acceleration[1]]
        if distance[0] > distance_acceleration[0] or distance[1] > distance_acceleration[1]:
            t_needed = max((2 * self.max_velocity[0] / self.max_acceleration[0] + (
                        distance[0] - distance_acceleration[0]) / self.max_velocity[0]), (
                                       2 * self.max_velocity[1] / self.max_acceleration[1] + (
                                           distance[1] - distance_acceleration[1]) / self.max_velocity[1]))
            possible = t_between > t_needed
        else:
            possible = True

        return possible

    def create_path(self, last: Point, destination: Point):
        # first step: linear

        last_index = len(self.path.points) - 2
        t = destination.time - last.time
        distance = [abs(last.x - destination.x), abs(last.y - destination.y)]

        vx = -0.5*(sqrt((-self.max_acceleration[0])*(-self.max_acceleration[0]*t*t + 4*distance[0]))-self.max_acceleration[0]*t)
        vy = -0.5*(sqrt((-self.max_acceleration[1])*(-self.max_acceleration[1]*t*t + 4*distance[1]))-self.max_acceleration[1]*t)

        vx_max = max(-self.max_velocity[0], min(vx, self.max_velocity[0]))
        vy_max = max(-self.max_velocity[1], min(vy, self.max_velocity[1]))

        tx_to_max = vx_max / self.max_acceleration[0]
        ty_to_max = vy_max / self.max_acceleration[1]
        t = 0  # for making the next lines more comfortable

        # STARTING POINT
        if tx_to_max > ty_to_max:
            # x component is limiting --> ax = maximum, ay needs to be adapted
            # ty_to_max must be equal to tx_to_max!
            t = tx_to_max
            ax = self.max_acceleration[0]
            ay = vy_max / t
        else:
            # y component limits
            t = ty_to_max
            ax = vx_max / t
            ay = self.max_acceleration[1]

        control = [ax, ay, last.time]
        self.controls.insert(last_index + 1, control)

        # ACCELERATION TO ZERO POINT
        control = [0.0, 0.0, time.time()+t]
        self.controls.insert(last_index + 2, control)

        # BRAKING POINT
        control = [-ax, -ay, destination.time - t]
        self.controls.insert(last_index + 3, control)

        # STOP POINT
        control = [0.0, 0.0, destination.time]
        self.controls.insert(last_index + 4, control)

import time
from God import God
from SpaceFree2D import SpaceSimulation2D
import matplotlib.pyplot as plt
from Obstacles2D import Obstacles2D


def start_simulation():
    g = God(20, 75)     # time between each data point, time between each controller input (equidistant in time)
    g.file_read()
    g.simulate() # cubic spline interpolation, equidistant controller
    s = SpaceSimulation2D(g)  # constructor(height of space in metres, god) --> WHAT DOES "HEIGHT OF SPACE" MEAN?

    ax = []
    ay = []
    sx = []
    sy = []
    vx = []
    vy = []
    t = []

    for data in g.calculation:
        print(data)
        t.append(data[1])
        sx.append(data[2])
        sy.append(data[3])
        vx.append(data[4])
        vy.append(data[5])
        ax.append(data[-1])
        ay.append(data[-2])

    #plt.plot(t, ax)
    #plt.plot(t, ay)
    #plt.plot(t, sx)
    #plt.plot(t, sy)
    #plt.plot(sx, sy)
    #plt.plot(t, vx)
    #plt.plot(t, vy)
    #plt.show()

    s.create_space()


if __name__ == "__main__":
    start_simulation()

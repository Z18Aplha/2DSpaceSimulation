from God import God
from SpaceFree2D import SpaceSimulation2D
from SpaceFree2D_OpenGL import SpaceFree2DOpenGL
import json
import imageio
import matplotlib.pyplot as plt
import numpy as np
from Obstacles2D import Obstacles2D


def start_simulation():
    parameters = json.load(open("Parameters.json"))
    g = God(parameters)     # time between each data point, time between each controller input (equidistant in time)
    g.file_read()
    g.simulate() # cubic spline interpolation, equidistant controller
    #s = SpaceSimulation2D(g)  # constructor(height of space in metres, god) --> WHAT DOES "HEIGHT OF SPACE" MEAN?
    s2 = SpaceFree2DOpenGL(g)

    ax = []
    ay = []
    sx = []
    sy = []
    vx = []
    vy = []
    t = []

    print("Without Channel:")
    for data in g.calculation:
        print(data)
        #t.append(data[1])
        #sx.append(data[2])
        #sy.append(data[3])
        #vx.append(data[4])
        #vy.append(data[5])
        #ax.append(data[-1])
        #ay.append(data[-2])

    print("With Channel:")
    for data in g.simulation:
        print(data)

    print("Controller Data:")
    for data in g.controller_data:
        print(data)

    #plt.plot(t, ax)
    #plt.plot(t, ay)
    #plt.plot(t, sx)
    #plt.plot(t, sy)
    #plt.plot(sx, sy)
    #plt.plot(t, vx)
    #plt.plot(t, vy)

    #plt.show()

    #s.create_space()
    s2.create_space()

    im = 0
    writer = imageio.get_writer('animation.gif', fps=s2.fps)
    for i in range(s2.counter):
        im = imageio.imread('video/'+str(i)+'.png')
        writer.append_data(im)
    for i in range(2*s2.fps):
        writer.append_data(im)
    writer.close()
    t = np.linspace(0, len(s2.time), len(s2.time))
    plt.plot(t, s2.time)
    plt.show()
    plt.plot(t, s2.currentfps)
    plt.show()


if __name__ == "__main__":
    start_simulation()


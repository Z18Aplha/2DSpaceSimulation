from God import God
from SpaceFree2D_OpenGL import SpaceFree2DOpenGL
import json
import imageio
import matplotlib.pyplot as plt
import numpy as np
import Lib as lib
from Obstacles2D import Obstacles2D
import pyglet


def start_simulation():
    parameters = json.load(open("Parameters.json"))
    g = God(parameters)
    g.file_read()
    g.simulate()
    s = SpaceFree2DOpenGL(g)

    for c in g.cars:
        plt.plot(c.liste)
    plt.show()

    for data in lib.data:
        time, obj, x, y, dir = data
        print(f"{float(time):< 6.4}    {obj:<10}     {float(x):< 6.4}     {float(y):< 10.3}      {float(dir): < 3.3}")

    s.create_space()


if __name__ == "__main__":
    start_simulation()

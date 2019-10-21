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
    parameters = json.load(open("OneCar.json"))
    g = God(parameters)
    g.file_read()
    g.simulate()
    s = SpaceFree2DOpenGL(g)

    for data in lib.data:
        time, obj, x, y, v, dir = data
        # print(f"{float(time):< 6.4}    {obj:<10}     {float(x):< 6.4}     {float(y):< 10.3}      {float(dir): < 3.3}")

    t = np.asarray(g.cars[0].planner.t_equi_in_t)

    planner_pos_x = np.asarray(g.cars[0].planner.path_from_v_equi_in_t.real)

    planner_vel_x = np.asarray(g.cars[0].planner.velocity_from_v_equi_in_t.real)

    planner_acc_x = np.asarray(g.cars[0].planner.acceleration_from_v_equi_in_t.real)

    planner_pos_y = np.asarray(g.cars[0].planner.path_from_v_equi_in_t.imag)

    planner_vel_y = np.asarray(g.cars[0].planner.velocity_from_v_equi_in_t.imag)

    planner_acc_y = np.asarray(g.cars[0].planner.acceleration_from_v_equi_in_t.imag)

    sim_pos_x = np.asarray(g.cars[0].position_x)

    sim_pos_y = np.asarray(g.cars[0].position_y)

    acc_vel_x = []
    for i in range(len(planner_vel_x)):
        try:
            acc_vel_x.append(planner_acc_x[i] * lib.pt/1000 + acc_vel_x[i-1])
        except IndexError:
            acc_vel_x.append(planner_acc_x[i] * lib.pt/1000)

    acc_vel_y = []
    for i in range(len(planner_vel_y)):
        try:
            acc_vel_y.append(planner_acc_y[i] * lib.pt / 1000 + acc_vel_y[i - 1])
        except IndexError:
            acc_vel_y.append(planner_acc_y[i] * lib.pt / 1000)

    acc_pos_x = []
    for i in range(len(planner_acc_x)):
        try:
            acc_pos_x.append(0.5 * planner_acc_x[i] * ((lib.pt / 1000) ** 2) + acc_vel_x[i] * (lib.pt/1000) + acc_pos_x[i - 1])
        except IndexError:
            acc_pos_x.append(0.5 * planner_acc_x[i] * ((lib.pt / 1000) ** 2) + acc_vel_x[i] * (lib.pt/1000) + g.cars[0].spawn[0])

    acc_pos_y = []
    for i in range(len(planner_acc_y)):
        try:
            acc_pos_y.append(0.5 * planner_acc_y[i] * ((lib.pt / 1000) ** 2) + acc_vel_y[i] * (lib.pt/1000) + acc_pos_y[i - 1])
        except IndexError:
            acc_pos_y.append(0.5 * planner_acc_x[i] * ((lib.pt / 1000) ** 2) + acc_vel_y[i] * (lib.pt / 1000) + g.cars[0].spawn[1])

    # plt.show()
    #
    # plt.plot(acc_pos_x - planner_pos_x, label='diff x int')
    # plt.plot(acc_pos_y - planner_pos_y, label='diff y int')
    # plt.plot(sim_pos_x - planner_pos_x, label='diff x sim')
    # plt.plot(sim_pos_y - planner_pos_y, label='diff y sim')
    # plt.legend()
    # plt.show()
    #plt.plot(acc_pos_x, acc_pos_y, label='integrated')
    #plt.plot(planner_pos_x, planner_pos_y, label='planner')
    plt.plot(sim_pos_x, sim_pos_y, '.', label='simulated')

    #plt.show()

    path_x = []
    path_y = []

    car = g.cars[0]

    for data in car.planner.acceleration_from_v_equi_in_t:
        x, y = car.test_dc_motor(data.real, data.imag)
        path_x.append(x+car.spawn[0])
        path_y.append(y+car.spawn[1])

    plt.plot(path_x, path_y, label='DC')


    px = []
    py = []
    i = 0
    for p_x, p_y in zip(path_x, path_y):
        if i == 30:
            px.append(p_x)
            py.append(p_y)
            i = 0
        i += 1

    #plt.plot(px, py, 'x', label='DC')
    plt.title('Path')
    plt.legend()
    plt.show()
    s.create_space()


if __name__ == "__main__":
    start_simulation()

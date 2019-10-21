from Point import Point
import math as m
import numpy as np
import sympy as sy
from scipy import signal

eventqueue = None
dt = None
ct = None
pt = None
fps = None
latency = None
carList = []
data = []
vis_data = []
collision = None
coll_det_freq = None
car_count = None
k_p = None
k_d = None
last_timestamp = None
statespace = None


# some setter methods
def set_eventqueue(eq):
    global eventqueue
    eventqueue = eq


def set_dt(t):
    global dt
    dt = t


def set_ct(t):
    global ct
    ct = t


def set_pt(t):
    global pt
    pt = t


def set_fps(framerate):
    global fps
    fps = framerate


def set_collision(c):
    global collision
    collision = c


def set_coll_det_freq(cdf):
    global coll_det_freq
    coll_det_freq = cdf


def set_latency(l):
    global latency
    latency = l


def set_carcount(c):
    global car_count
    car_count = c


def set_k_p(kp):
    global k_p
    k_p = kp


def set_k_d(kd):
    global k_d
    k_d = kd


def set_statespace(ss):
    global statespace
    statespace = ss


# some methods used my multiple classes
def angle(p1: Point, p2: Point):
    phi = m.atan2(p2.y - p1.y, p2.x - p1.x)
    return phi


def distance(p1: Point, p2: Point):
    x = p2.x - p1.x
    y = p2.y - p1.y
    d = np.linalg.norm([x, y])
    return d

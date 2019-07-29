from Point import Point
import math as m
import numpy as np

eventqueue = None
dt = None
carList = []
data = []
collision = None
coll_det_freq = None


# some setter methods
def set_eventqueue(eq):
    global eventqueue
    eventqueue = eq


def set_dt(t):
    global dt
    dt = t


def set_collision(c):
    global collision
    collision = c


def set_coll_det_freq(cdf):
    global coll_det_freq
    coll_det_freq = cdf


# some methods used my multiple classes
def angle(p1: Point, p2: Point):
    phi = m.atan2(p2.y - p1.y, p2.x - p1.x)
    return phi


def distance(p1: Point, p2: Point):
    x = p2.x - p1.x
    y = p2.y - p1.y
    d = np.linalg.norm([x, y])
    return d

#import os
# import pygame
from math import tan, radians, degrees, copysign, sqrt
import time
import Controller
from God import God

g= God()
g.file_read()
i = 0
while 1 < 2:
    i = i + 1
    car1.update()
    if (i % 10000) == 0:  # skips showing some update routines - better overview
        #print(car1.status())
        print(time.time(), car1.position[0], car1.position[1], car1.acceleration[0], car1.acceleration[1])

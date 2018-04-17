import numpy as np
from math import cos, acos, sin, tan, pi, sqrt
import matplotlib.pyplot as plt
import random
import robot_func

expc3 = [1,2,3]

[omghat, theta] = robot_func.AxisAng3(expc3)

R = [[0,-1,0],[0,0,-1],[1,0,0]]
invR = robot_func.RotInv(R)
print invR

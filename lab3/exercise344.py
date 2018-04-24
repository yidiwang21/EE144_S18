#!/usr/bin/env python
import exercise343 as rotmat
import modern_robotics as mr
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

def TransDecide(T):
    try:
        if np.shape(T) == (4,4):
            R = [[T[0][0],T[0][1],T[0][2]], [T[1][0],T[1][1],T[1][2]], [T[2][0],T[2][1],T[2][2]]]
            p = [T[0][3],T[1][3],T[2][3]]
            if rotmat.RotDecide(R) == True:
                return True
        else:
            return False
    except:
        print("Failed to calculate!")

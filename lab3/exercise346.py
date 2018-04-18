#!/usr/bin/env python
import rotmat
import exercise345 as ex
import modern_robotics as mr
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

def se3Decide(se3mat):
    try:
        if np.shape(se3mat) == (4,4):
            so3mat = [[se3mat[0][0],se3mat[0][1],se3mat[0][2]],
                    [se3mat[1][0],se3mat[1][1],se3mat[1][2]],
                    [se3mat[2][0],se3mat[2][1],se3mat[2][2]]]
            print so3mat
        if np.shape(se3mat) == (4,4) and ex.so3Decide(so3mat) == True:
            pass
        if se3mat[3][0] == 0 and se3mat[3][1] == 0 and se3mat[3][2] == 0 and se3mat[3][3] == 0:
            return True
        else:
            return Flase
    except:
        print("Failed to calculate!")

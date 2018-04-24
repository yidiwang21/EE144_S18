#!/usr/bin/env python
import modern_robotics as mr
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

def so3Decide(so3mat):
    try:
        if np.shape(so3mat) == (3,3):
            if so3mat[0][0] == 0 and so3mat[1][1] == 0 and so3mat[2][2] == 0
                if so3mat[0][1] == -so3mat[1][0] and so3mat[0][2] == -so3mat[2][0] and so3mat[1][2] == -so3mat[2][1]:
                    return True
        else:
            return False
    except:
        print("Failed to calculate!")

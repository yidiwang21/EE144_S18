#!/usr/bin/env python
import modern_robotics as mr
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

epsilon = 0.05
DET_MAX = 1 + epsilon
DET_MIN = 1 - epsilon

def RotDecide(R):
    try:
        if np.linalg.det(R) <= DET_MAX and np.linalg.det(R) >= DET_MIN:
            pass
        if np.matrix.trace(np.linalg.inv(R)) - np.matrix.trace(mr.RotInv(R)) <= epsilon:
            return True
        else:
            return False
    except:
        print("Failed to calculate!")

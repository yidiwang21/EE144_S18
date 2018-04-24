#!/usr/bin/env python
import modern_robotics as mr
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt
from scipy.linalg import lu

epsilon = 0.05
DET_MAX = 1 + epsilon
DET_MIN = 1 - epsilon

def MatDiag(A):
    pl, u = lu(A, permute_l=True)
    return u

def RotDecide(R):
    try:
        if np.shape(R) == (4,4):
            if np.linalg.det(R) <= DET_MAX and np.linalg.det(R) >= DET_MIN:
                # FIXME: whether need to be diagalised
                # if np.matrix.trace(MatDiag(np.linalg.inv(R))) - np.matrix.trace(MatDiag(mr.RotInv(R))) <= epsilon:
                if np.matrix.trace(np.linalg.inv(R)) - np.matrix.trace(mr.RotInv(R)) <= epsilon:
                    return True
        else:
            return False
    except:
        print("Failed to calculate!")

# just for test
R = np.array([[0.8378, 0.0940, 0.5378],
            [0.0071, 0.9831, -0.1830],
            [-0.5459, 0.1572, 0.8230]])

print np.linalg.det(R), RotDecide(R), MatDiag(R), np.matrix.trace(MatDiag(R))

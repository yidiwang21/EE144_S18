import numpy as np
from math import cos, acos, sin, tan, pi, sqrt
import matplotlib.pyplot as plt
import random

DETERMINANT = 1.0000000000

# Computes the inverse of the rotation matrix R.
def RotInv(R):
    try:
        if det(R) == DETERMINANT:
            invR = [[R[0][0],R[1][0],R[2][0]],[R[0][1],R[1][1],R[2][1]],[R[0][2],R[1][2],R[2][2]]]
            return invR
        else:
            print("The determinant of the input matrice is not equal to 1!")
    except:
        print("Failed to calculate!")

# Returns the 3 × 3 skew-symmetric matrix corresponding to omg.
def VecToso3(omg):
    try:
        if len(omg) == 3:
            so3mat = [[0,-omg[2],omg[1]], [omg[2],0,-omg[0]], [-omg[1],omg[0],0]]
            return so3mat
        else:
            print("Illegal input!")
    except:
         print("Failed to calculate!")

# Returns the 3-vector corresponding to the 3×3 skew-symmetric matrix so3mat.
def so3ToVec(so3mat):
    try:
        if so3mat[0][0] == 0 and so3mat[1][1] == 0 and so3mat[2][2] == 0:
            pass
        if so3mat[0][1] == -so3mat[1][0] and so3mat[0][2] == -so3mat[2][0] and so3mat[1][2] == -so3mat[2][1]:
            omg = [so3mat[2][1],so3mat[0][2],so3mat[1][0]]
            return omg
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

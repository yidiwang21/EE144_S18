#!/usr/bin/env python
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt
import matplotlib.pyplot as plt
import random

DET_MAX = 1.05
DET_MIN = 0.95

# Computes the inverse of the rotation matrix R.
def RotInv(R):
    try:
        if np.linalg.det(R) <= DET_MAX and np.linalg.det(R) >= DET_MIN:
            invR = [[R[0][0],R[1][0],R[2][0]],[R[0][1],R[1][1],R[2][1]],[R[0][2],R[1][2],R[2][2]]]
            return invR
        else:
            print("The determinant of the input matrice is not equal to 1!")
    except:
        print("Failed to calculate!")

# Returns the 3x3 skew-symmetric matrix corresponding to omg
def VecToso3(omg):
    try:
        if len(omg) == 3:
            so3mat = [[0,-omg[2],omg[1]], [omg[2],0,-omg[0]], [-omg[1],omg[0],0]]
            return so3mat
        else:
            print("Illegal input!")
    except:
         print("Failed to calculate!")

# Returns the 3-vector corresponding to the 3x3 skew-symmetric matrix so3mat
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

# Extracts the rotation axis omega and the rotation amount theta from the 3-vector of exponential coordinates for rotation, expc3
def AxisAng3(expc3):
    try:
        if len(expc3) == 3:
            theta = np.linalg.norm(expc3)
            omghat = expc3 / theta
            return (omghat, theta)
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# Compute the rotation matrix R in SO(3) corresponding to the matrix exponential of so3mat in so(3)
# Rodriguez R = I + sin(theta)*omghat + (1-cos(theta))*omghat^2
def MatrixExp3(so3mat):
    try:
        if len(so3mat) == 3:
            [omghat, theta] = AxisAng3(expc3)
            R = np.eye(3) + np.matmul(np.sin(theta), VecToso3(omghat)) + np.matmul((1 - np.cos(theta)), VecToso3(omghat), VecToso3(omghat))
            return R
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# TODO: shit!!!
# Computes the matrix logarithm so3mat in so(3) of the rotation matrix R in SO(3)
def MatrixLog3(R):
    try:
        if np.linalg.det(R) <= DET_MAX and np.linalg.det(R) >= DET_MIN and np.shape(R) == (3,3):
            pass

        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# Builds the homogeneous transformation matrix T corresponding to a rotation matrix R in SO(3) and a position vector p in R^3
def RpToTrans(R, p):
    try:
        if len(p) == 3 and np.shape(R) == (3,3) and np.linalg.det(R) <= DET_MAX and np.linalg.det(R) >= DET_MIN:
            T = [[R[0][0],R[0][1],R[0][2],p[0]] , [R[1][0],R[1][1],R[1][2],p[1]], [R[2][0],R[2][1],R[2][2],p[2]] , [0,0,0,1]]
            return T
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# Extracts the rotation matrix and position vector from a homogeneous transfor-mation matrix T.
def TransToRp(T):
    try:
        if np.shape(T) == (4,4):
            R = [[T[0][0],T[0][1],T[0][2]], [T[1][0],T[1][1],T[1][2]], [T[2][0],T[2][1],T[2][2]]]
            p = [T[0][3],T[1][3],T[2][3]]
            if np.linalg.det(R) <= DET_MAX and np.linalg.det(R) >= DET_MIN:
                return (R, p)
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# Computes the inverse of a homogeneous transformation matrix T.
def TransInv(T):
    try:
        if np.shape(T) == (4,4):
            [R, p] = TransToRp(T)
            invR = RotInv(R)
            pt = -np.matmul(invR, p)
            invT = [[invR[0][0],invR[0][1],invR[0][2],pt[0]],
                    [invR[1][0],invR[1][1],invR[2][2],pt[1]],
                    [invR[2][0],invR[2][1],invR[2][2],pt[2]],
                    [0,0,0,1]]
            return invT
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# Returns the se(3) matrix corresponding to a 6-vector twist V
def VecTose3(V):
    try:
        if (len(V)==6):
            so3mat = VecToso3([V[0],V[1],V[2]])
            se3mat = [[so3mat[0][0],so3mat[0][1],so3mat[0][2],V[3]],
                    [so3mat[1][0],so3mat[1][1],so3mat[1][2],V[4]],
                    [so3mat[2][0],so3mat[2][1],so3mat[2][2],V[5]] ,
                    [0,0,0,0]]
            return se3mat
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# Returns the 6-vector twist corresponding to an se(3) matrix se3mat
def se3ToVec(se3mat):
    try:
        if np.shape(se3mat) == (4,4):
            V = [se3mat[2][1],se3mat[0][2],se3mat[1][0], se3mat[0][3],se3mat[1][3],se3mat[2][3]]
            return V
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# Computes the 6x6 adjoint representation [AdT] of the homogeneous transformation matrix T
def Adjoint(T):
    try:
        if np.shape(T) == (4,4):
            [R, p] = TransToRp(T)
            if np.linalg.det(R) <= DET_MAX and np.linalg.det(R) >= DET_MIN:
                pass
            pso = VecToso3(p)
            Rt = np.matmul(pso, R)
            AdT = [[R[0][0],R[0][1],R[0][2],0,0,0],
                [R[1][0],R[1][1],R[1][2],0,0,0],
                [R[2][0],R[2][1],R[2][2],0,0,0],
                [Rt[0][0],Rt[0][1],Rt[0][2],R[0][0],R[0][1],R[0][2]],
                [Rt[1][0],Rt[1][1],Rt[1][2],R[1][0],R[1][1],R[1][2]],
                [Rt[2][0],Rt[2][1],Rt[2][2],R[2][0],R[2][1],R[2][2]]]
            return AdT
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# Returns a normalized screw axis representation S of a screw described by a unit vector s in the direction of the screw axis,
# located at the point q, with pitch h.
def SkewToAxis(q, s, h):
    try:
        if len(q) == 3 and len(s) == 3:
            sq = np.cross(s,q)
            S = [[s[0]],
                [s[1]],
                [s[2]],
                [(-sq[0]+h*s[0])],
                [(-sq[1]+h*s[1])],
                [(-sq[2]+h*s[2])]]
            return S
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# TODO: uncertain about the computations
# Extracts the normalized screw axis S and the distance traveled along the screw theta
# from the 6-vector of exponential coordinates Stheta
def AxisAng6(expc6):
    try:
        if len(expc6) == 6:
            theta = np.linalg.norm(expc6)
            S = expc6 / theta
            return (S, theta)
        else:
            print("Illegal input!")
    except:
        print("Failed to calculate!")

# TODO: don't know the formula
# Computes the homogeneous transformation matrix T in SE(3) corresponding to
# the matrix exponential of se3mat in se(3)
# Rodriguez R = I + sin(theta)*omg + (1-cos(theta))*omg^2
def MatrixExp6(se3mat):
    pass

# TODO: shit!!!
# Computes  the matrix logarithm se3mat in se(3) of the homogeneous transformation matrix T in SE(3)
def MatrixLog6(T):
    pass

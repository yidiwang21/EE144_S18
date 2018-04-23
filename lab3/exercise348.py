#!/usr/bin/env python
import modern_robotics as mr
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt

PI = 3.1415926535897

# compute the corresponding normalized screw axis
q = np.array([0,2,0])
s = np.array([0,0,1])
h = 2
theta = PI
T = np.array([[1,0,0,2],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]])

S = mr.ScrewToAxis(q,s,h)
print S
S1, theta1 = mr.AxisAng6(S)
print S1, theta1
# FIXME: how to use theta???   
se3mat = mr.VecTose3(S1)
Travel = mr.MatrixExp6(se3mat)
T1 = np.matmul(Travel,T)
print T1

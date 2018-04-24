#!/usr/bin/env python
import modern_robotics as mr
import numpy as np
from math import cos, acos, sin, tan, pi, sqrt
import matplotlib.pyplot as plt

PI = 3.1415926535897

# just for test
q = np.array([0,2,0])
s = np.array([0,0,1])
h = 2
theta = PI
T = np.array([[1,0,0,2],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]])

# start here
angle = [theta/4, theta/2, theta*3/4, theta]
S = mr.ScrewToAxis(q,s,h)

def computeConfig(S, angle):
    try:
        Sn = S * angle
        T0n = mr.VecTose3(Sn)
        Tn = np.matmul(mr.MatrixExp6(T0n),T)
        return Tn
    except:
        print("Failed to calculate!")

if __name__ == '__main__':
    try:
        i = 0
        while i < 4:            # iter 4 times to calculate intermedian configurations
            Tn = computeConfig(S,angle[i])
            print 'Tn[', i, '] = ', '\n', Tn
            i = i + 1
    except KeyboardInterrupt:
        exit()

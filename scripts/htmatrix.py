import numpy as np
from numpy import sin, cos
import math

class HTMatrix():
    
    def __init__(self):
        self.d2r = math.pi/180
        self.r2d = 180/math.pi
        
    def trans(self, x, y, z):
        mat = np.matrix([[1, 0, 0, x],
                        [0, 1, 0, y],
                        [0, 0, 1, z],
                        [0, 0, 0, 1]])
        return mat

    def rotx(self, theta):
        cth = cos(theta)
        sth = sin(theta)
        mat = np.matrix([[1, 0, 0, 0],
                        [0, cth, -sth, 0],
                        [0, sth, cth, 0],
                        [0, 0, 0, 1]])
        return mat

    def roty(self, theta):
        cth = cos(theta)
        sth = sin(theta)
        mat = np.matrix([[cth, 0, sth, 0],
                        [0, 1, 0, 0],
                        [-sth, 0, cth, 0],
                        [0, 0, 0, 1]])
        return mat

    def rotz(self, theta):
        cth = cos(theta)
        sth = sin(theta)
        mat = np.matrix([[cth, -sth, 0, 0],
                        [sth, cth, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        return mat

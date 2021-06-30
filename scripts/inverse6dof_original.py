#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from numpy import sin, cos
from htmatrix import HTMatrix
htmat = HTMatrix()

class IK6dof():
    def __init__(self):
        self.l = [9.5, 10.5, 9.5, 15]
        self.def_th = [90, 145, 45, 45]
        
    def forward(self, th):
        l1, l2, l3, l4 = self.l
        th1, th2, th3, th4 = th
        th1 = th1 * htmat.d2r
        th2 = th2 * htmat.d2r
        th3 = th3 * htmat.d2r
        th4 = th4 * htmat.d2r

        t01 = htmat.trans(0, 0, l1) * htmat.rotz(th1)
        t02 = t01 * htmat.rotx(90*htmat.d2r) * htmat.rotz(th2)
        t03 = t02 * htmat.trans(l2, 0, 0) * htmat.rotz(th3)
        t04 = t03 * htmat.rotz(-135*htmat.d2r) * htmat.trans(l3, 0, 0) * htmat.rotz(th4)
        t05 = t04 * htmat.rotz(-135*htmat.d2r) * htmat.trans(l4, 0, 0)

        joint1 = t01[:3, 3]
        joint2 = t02[:3, 3]
        joint3 = t03[:3, 3]
        joint4 = t04[:3, 3]
        joint5 = t05[:3, 3]
        return joint5.flatten()

    def calc_jacob(self, th1, th2, th3, th4):
        l1, l2, l3, l4 = self.l

        s1 = sin(th1)
        s2 = sin(th2)
        s3 = sin(th3)
        s4 = sin(th4)
        c1 = cos(th1)
        c2 = cos(th2)
        c3 = cos(th3)
        c4 = cos(th4)
        s23 = sin(th2) * cos(th3) + cos(th2) * sin(th3)
        c23 = cos(th2) * cos(th3) - sin(th2) * sin(th3)
        c34 = cos(th3) * cos(th4) - sin(th3) * sin(th4)
        s34 = sin(th3) * cos(th4) + cos(th3) * sin(th4)

        j11 = -s1*(l2*c2 + l3*(s23 - c23) - 2*l4*(c2*s34 + s2*c34))
        j12 = c1*(-l2*s2 + l3*(c23 + c23) - 2*l4*(c2*s34 + s2*c34))
        j13 = c1*(l3*(s23 + c23) - 2*l4*(c2*c34 - s2*s34))
        j14 = -2*c1*l4*(c2*c34 - s2*s34)
        j21 = c1*(l2*c2 + l3*(s23 - c23) - 2*l4*(c2*s34 + s2*c34))
        j22 = s1*(-l2*s2 + l3*(c23 + c23) - 2*l4*(-s2*s34 + c2*c34))
        j23 = s1*(l3*(s23 + c23) - 2*l4*(c2*c34 - s2*s34))
        j24 = 2*s1*l4*(c2*c34 - s2*c34)
        j31 = 0
        j32 = l2*c2 - l3*(-s23 + c23) - 2*l4*(s2*c34 + c2*s34)
        j33 = -l3*(-s23 + c23) - 2*l4*(c2*c34 + s2*c34)
        j34 = -2*l4*(c2*s34 + s2*c34)

        jacob = np.matrix(((j11, j12, j13, j14),
                           (j21, j22, j23, j24),
                           (j31, j32, j33, j34)))
        return jacob
    
    def check_th(self, th):
        if sum(np.array(th) < 0) or sum(np.array(th) > 180):
            print('invalid angle: {}'.format(np.round(th,1)))
            print('setting initialize angle')
            th = self.def_th
            self.valid_flag = False
        else:
            self.valid_flag = True
        return th
    
    def finish_info(self, start_pos, start_th, crt_pos, th):
        print('start, pos :{}, theta:{}'.format(np.round(start_pos,1), np.round(np.array(start_th))))
        print('goal,  pos :{}, theta:{}'.format(np.round(crt_pos,1), np.round(np.array(th))))  
    
    def inverse(self, pos, th):
        x, y, z = pos
        th1, th2, th3, th4 = th
        th1 = th1 * htmat.d2r
        th2 = th2 * htmat.d2r
        th3 = th3 * htmat.d2r
        th4 = th4 * htmat.d2r
        start_th = th
        start_pos = np.array(self.forward(th))
        goal_pos = np.matrix(((x), (y), (z)))

        while 1:
            crt_pos = self.forward(th)

            if abs(goal_pos-crt_pos).mean() < 0.01:
                th = self.check_th(th)
                self.finish_info(start_pos, start_th, crt_pos, th)
                break

            j = self.calc_jacob(th1, th2, th3, th4)

            invj = np.linalg.pinv(j)

            dx = goal_pos - crt_pos
            dth = 0.02 * invj * dx.reshape(3,1)
            th1 = th1 + dth[0, 0]
            th2 = th2 + dth[1, 0]
            th3 = th3 + dth[2, 0]
            th4 = th4 + dth[3, 0]

            th = [th1*htmat.r2d, th2*htmat.r2d, th3*htmat.r2d, th4*htmat.r2d]

        return th, crt_pos

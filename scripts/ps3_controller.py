#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from armrobot_py.msg import AngleArray

class PS3Controller():
    def __init__(self):
        rospy.init_node('ps3_controller')
        rospy.Subscriber('/joy', Joy, self.controller_callback)
        self.pub_angle = rospy.Publisher('angle', AngleArray, queue_size=1)
        self.msg = AngleArray()
        self.buttons_name = ["cross", "circle", "delta", "square", "L1", "R1", "L2", "R2", "SELECT", "START", "PS", "L3", "R3", "up", "down", "left", "right"]
        self.axes_name = ["stick left", "stick left", "L2", "stick right", "stick right", "R2"]
        self.past_cmd = [0, 0, 0, 0, 1.0, 1.0]
        self.th = [0, 0, 0, 0, 0, 0]

    def controller_callback(self, cmd):
        if cmd.axes[0] - self.past_cmd[0] > 0:
            self.th[0] = -cmd.axes[0]*10
        else:
            self.th[0] = 0
        self.past_cmd[0] = cmd.axes[0]
        print(self.th[0])
        # th[1] = cmd.axes[1]*10
        # th[2] = -cmd.axes[3]*10
        # th[3] = cmd.axes[4]*10
        # th[4] = 180
        # th[5] = 100

    def publish(self):
        self.msg.th = self.th
        self.pub_angle.publish(self.msg)
        

if __name__ == '__main__':
    pc = PS3Controller()
    rospy.spin()

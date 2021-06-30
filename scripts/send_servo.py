#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from std_msgs.msg import String
from armrobot_py.msg import AngleArray
from armrobot_py.msg import KeysArray
from armrobot_py.srv import CalcInverse

class KeyServo():
    def __init__(self):
        rospy.init_node('keys_to_servo')
        rospy.wait_for_service('calculation_ik')
        self.crt_th = [90, 145, 45, 45]
        self.wrist_th = 180
        self.grip_th = 100
        self.msg = AngleArray()
        self.sub_key = rospy.Subscriber('/keys', KeysArray, self.key_callback)
        self.sub_com = rospy.Subscriber('/command', String, self.com_callback)
        self.pub_angle = rospy.Publisher('angle', AngleArray, queue_size=1)
        self.client = rospy.ServiceProxy('calculation_ik', CalcInverse)
        self.req = CalcInverse()
            

    def com_callback(self, command):
        if command.data == "open":
            self.grip_th = 0
        elif command.data == "close":
            self.grip_th = 100
        elif command.data == "cw":
            self.wrist_th = 180
        elif command.data == "ccw":
            self.wrist_th = 90
        self.command_pub()

    def key_callback(self, data):
        self.req.pos = data.keys
        self.req.th = self.crt_th
        result = self.client(self.req.pos, self.req.th)
        self.goal_th = result.res
        if result.flag:
            self.publisher()
            self.crt_th = self.goal_th
    
    def publisher(self):
        r = rospy.Rate(100)

        th_list = self.generate_orbit()
        
        for th1, th2, th3, th4 in zip(th_list[0], th_list[1], th_list[2], th_list[3]):
            self.msg.th = [th1, th2, th3, th4, self.wrist_th, self.grip_th]
            self.pub_angle.publish(self.msg)
            r.sleep()
    
    def command_pub(self):
        r = rospy.Rate(6)
        self.msg.th = [self.crt_th[0], self.crt_th[1], self.crt_th[2], self.crt_th[3], self.wrist_th, self.grip_th]
        self.pub_angle.publish(self.msg)
        r.sleep()

    def generate_orbit(self):
        move_time = 1.5
        accel_time_rate = 0.3
        accel_time = move_time*accel_time_rate
        th_list = []
        for crt, goal in zip(self.crt_th, self.goal_th):
            tmp = []
            diff_angle = goal - crt
            omega_max = diff_angle / (move_time*(1-accel_time_rate))
            accel = omega_max / accel_time
            for now_time in np.arange(0, move_time, 0.01):
                if now_time < accel_time:
                    angle = crt
                    angle = angle + omega_max*now_time**2 / (2*accel_time)
                elif accel_time <= now_time and now_time <= (move_time-accel_time):
                    angle = crt + (omega_max*accel_time/2)
                    angle = angle + omega_max*(now_time-accel_time)
                else:
                    deceleration_time = (now_time-(move_time-accel_time))
                    angle = crt + (omega_max*accel_time/2) + omega_max*(move_time-2*accel_time)
                    angle = angle + (2*omega_max - omega_max*deceleration_time/accel_time)*deceleration_time/2
                tmp.append(angle)
            th_list.append(tmp)
        return th_list

if __name__ == '__main__':
    KeyServo()
    rospy.spin()

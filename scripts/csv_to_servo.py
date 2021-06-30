#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from std_msgs.msg import String, Bool
from armrobot_py.msg import AngleArray, KeysArray
from armrobot_py.srv import CalcInverse, Keys, Command, KeysResponse, CommandResponse

class KeyServo():
    def __init__(self):
        rospy.init_node('keys_to_servo')
        rospy.wait_for_service('calculation_ik')
        self.crt_th = [90, 145, 45, 45]
        self.wrist_th = 180
        self.grip_th = 100
        self.angle_msg = AngleArray()
        self.pub_angle = rospy.Publisher('angle', AngleArray, queue_size=1)
        self.srv_key = rospy.Service('service_keys', Keys, self.key_callback)
        self.srv_com = rospy.Service('service_command', Command, self.com_callback)
        self.ik_client = rospy.ServiceProxy('calculation_ik', CalcInverse)
        self.msg_key = KeysResponse()
        self.msg_com = CommandResponse()
        self.ik_req = CalcInverse()
            

    def com_callback(self, data):
        if data.command == "open":
            print('open gripper')
            self.grip_th = 0
        elif data.command == "close":
            print('close gripper')
            self.grip_th = 95
        elif data.command == "cw":
            print('rotate clock wise')
            self.wrist_th = 180
        elif data.command == "ccw":
            print('rotate counter clock wise')
            self.wrist_th = 90
        else:
            print('none')
        self.command_pub()
        self.msg_com.success = True
        return self.msg_com

    def key_callback(self, data):
        self.ik_req.pos = data.pos
        self.ik_req.th = self.crt_th
        result = self.ik_client(self.ik_req.pos, self.ik_req.th)
        self.goal_th = result.res
        if result.flag:
            self.publisher()
            self.crt_th = self.goal_th
            self.msg_key.success = True
            return self.msg_key
    
    def publisher(self):
        r = rospy.Rate(240)

        th_list = self.generate_orbit()
        
        for th1, th2, th3, th4 in zip(th_list[0], th_list[1], th_list[2], th_list[3]):
            self.angle_msg.th = [th1, th2, th3, th4, self.wrist_th, self.grip_th]
            self.pub_angle.publish(self.angle_msg)
            r.sleep()
    
    def command_pub(self):
        r = rospy.Rate(10)
        self.angle_msg.th = [self.crt_th[0], self.crt_th[1], self.crt_th[2], self.crt_th[3], self.wrist_th, self.grip_th]
        self.pub_angle.publish(self.angle_msg)
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

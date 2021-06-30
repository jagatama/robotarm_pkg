#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import String
from armrobot_py.msg import KeysArray

class KeyDriver():
    def __init__(self):
        rospy.init_node('keyboard_driver')
        self.pub_key = rospy.Publisher('keys', KeysArray, queue_size=1)
        self.pub_com = rospy.Publisher('command', String, queue_size=1)
        self.command = String()
        self.msg = KeysArray()
        self.publisher()

    def publisher(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            key = input('waiting command or position >')
            if key == 'open':
                self.command.data = 'open'
                rospy.loginfo('open gripper')
            elif key == 'close':
                self.command.data = 'close'
                rospy.loginfo('close gripper')
            elif key == 'cw':
                self.command.data = 'cw'
                rospy.loginfo('rotate clock wise')
            elif key == 'ccw':
                self.command.data = 'ccw'
                rospy.loginfo('rotate counter clock wise')
            elif key == 'send':
                self.command.data = 'send'
                rospy.loginfo('send all theta')
            elif key == 'exit':
                rospy.loginfo('please press ctrl+c')
                break
            else:
                x, y, z = (float(i) for i in key.split())
                if x < -15 or x > 15:
                    print('invalid x range')
                elif y < 0 or y > 15:
                    print('invalid y range')
                elif z < 0 or z > 45:
                    print('invalid z range')
                else:   
                    # rospy.loginfo('setting pos [{}, {}, {}]'.format(x, y, z))
                    self.msg.keys = [x, y, z]
                    self.command.data = 'pos'
                    self.pub_key.publish(self.msg)
            self.pub_com.publish(self.command)
            r.sleep()

if __name__ == '__main__':
    KeyDriver()
    rospy.spin()

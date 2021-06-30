#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import pandas as pd
import time
from armrobot_py.msg import KeysArray
from std_msgs.msg import String
from armrobot_py.srv import Keys, Command

class CsvToKeys():
    def __init__(self):
        rospy.init_node('csv_to_keys')
        self.path = '/home/yohsuke/ドキュメント/motion/'
        self.df = self.read_csv()
        rospy.wait_for_service('service_keys')
        rospy.wait_for_service('service_command')
        self.client_key = rospy.ServiceProxy('service_keys', Keys)
        self.client_com = rospy.ServiceProxy('service_command', Command)
        self.req_key = Keys()
        self.req_com = Command() 
        self.publisher()

    def read_csv(self):
        df = pd.read_csv(self.path+'test.csv')
        return df

    def publisher(self):
        time.sleep(2)
        for x, y, z, com in zip(self.df['x'], self.df['y'], self.df['z'], self.df['com']):
            if x + y + z > 90:
                self.req_com.command = com
                res = self.client_com(self.req_com.command)
                time.sleep(1)
            else:
                keys = [x, y, z]
                self.req_key.pos = [x, y, z]
                res = self.client_key(self.req_key.pos)
            

if __name__ == '__main__':
    CsvToKeys()
    rospy.spin()

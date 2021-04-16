#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
import numpy as np
from numpy import linalg
import matplotlib.pyplot as plt
import scipy  
from scipy import signal
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv

class Robo_Block():

    def __init__(self, des_t, pos_inc, car=1, curr_tension, curr_pos):
        # rospy.init_node("robo_block", anonymous=True)
        self.des_t = des_t
        self.pos_inc = pos_inc
        self.des_pos = None
        self.car = car
        self.curr_tension = curr_tension
        # self.ten_time = []
        self.curr_pos = curr_pos
        self.done = False
        self.error = None
        x = self.curr_pos[0]
        if x < 0:
            self.des_pos = x - self.pos_inc
        else:
            self.des_pos = x + self.pos_inc
        # rospy.Subscriber('/vrep_ros_interface/tensions', Float32MultiArray, self.ten_callback, queue_size = 1, buff_size=2**8)
        # rospy.Subscriber('/vrep_ros_interface/car_pos' + str(car), Float32MultiArray, self.pos1_callback, queue_size=1, buff_size=2**8)

        # while not self.done:
        #     self.control()
        # rospy.spin()

    # def ten_callback(self, msg):
    #     # print('time: ' + str(msg.data[4]))
    #     if self.car == 1:
    #         self.avg_tension.append(np.abs((msg.data[0] + msg.data[1]) / 2))
    #     else:
    #         self.avg_tension.append(np.abs((msg.data[2] + msg.data[3]) / 2))
    #     self.ten_time.append(msg.data[4])

    #     # self.control()

    # # car
    # def pos1_callback(self, msg):
    #     self.all_pos.append(msg.data)
    #     x = msg.data[0]
    #     if x < 0:
    #         self.des_pos = x - self.pos_inc
    #     else:
    #         self.des_pos = x + self.pos_inc
    
    def control(self):
        if self.avg_tension != [] and self.des_pos:
            error = self.des_t - self.avg_tension[-1]
            if error > 0.01:
                # TODO call car controller here
            else:
                # TODO call car controller to stop
                self.done = True


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
from controller import Controller

class Robo_Block():

    def __init__(self, des_t, car, curr_tension, curr_pos):
        # rospy.init_node("robo_block", anonymous=True)
        self.des_t = des_t
        # self.pos_inc = pos_inc
        self.des_pos = None
        self.car = car
        self.curr_tension = curr_tension
        self.controller = Controller(None, curr_pos, car)
        # self.ten_time = []
        self.curr_pos = curr_pos
        self.done = False
        self.error = None
        self.last_pos = None
        self.inc_sign = 1
        # if car == 1:
        #     self.K = 0.02
        # else:
        #     self.K = 0.2
        
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
    
    def control(self, maintain):
        x = self.curr_pos[0]
        if self.curr_tension:
            if not maintain:
                self.last_pos = self.curr_pos
                error = self.des_t - self.curr_tension[-1]
                # print('tension error: ' + str(error))
                if abs(error) > 0.05:
                    # print('calling controller from robo_block 1')
                    self.controller.des_tension = self.des_t
                    self.controller.curr_tension = self.curr_tension[-1]
                    self.controller.set_torque()
                    self.done = False
                else:
                    # print('calling controller from robo_block 2')
                    self.controller.des_tension = self.des_t
                    self.controller.curr_tension = self.curr_tension[-1]
                    self.controller.set_torque()
                    self.done = True
            else:
                self.controller.des_pos = self.last_pos[0]
                self.controller.curr_pos = self.curr_pos
                self.controller.set_velocity()
        # x = self.curr_pos[0]
        # if self.curr_tension:
        #     error = np.abs(self.des_t - self.curr_tension[-1])
        #     if np.abs(self.des_t) < np.abs(self.curr_tension[-1]):
        #         self.inc_sign = -0.2
        #     else:
        #         self.inc_sign = 1

        #     pos_inc = self.K*error*self.inc_sign

        #     if x < 0:
        #         self.des_pos = x - pos_inc
        #     else:
        #         self.des_pos = x + pos_inc
            
        #     if self.curr_tension != [] and self.des_pos:
        #         if not maintain:
        #             self.last_pos = self.curr_pos
        #             # error = self.des_t - self.curr_tension[-1]
        #             # print('tension error: ' + str(error))
        #             if abs(error) > 0.05:
        #                 # print('calling controller from robo_block 1')
        #                 self.controller.des_pos = self.des_pos
        #                 self.controller.curr_pos = self.curr_pos
        #                 self.controller.set_velocity()
        #                 self.done = False
        #             else:
        #                 # print('calling controller from robo_block 2')
        #                 self.controller.des_pos = self.curr_pos[0]
        #                 self.controller.curr_pos = self.curr_pos
        #                 self.controller.set_velocity()
        #                 self.done = True
        #         else:
        #             self.controller.des_pos = self.last_pos[0]
        #             self.controller.curr_pos = self.curr_pos
        #             self.controller.set_velocity()


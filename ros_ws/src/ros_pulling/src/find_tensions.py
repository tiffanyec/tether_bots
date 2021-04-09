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

class Tensions():

    def __init__(self):
        self.left_avg_tension = []
        self.right_avg_tension = []
        self.ten_time = []
        self.pos_time = []
        self.d = 0.5/2
        self.r = 0.3/2
        self.left_dist = 0
        self.M = 0.2
        # self.right_dist = 0
        self.car1_pos = []
        self.car2_pos = []
        self.block_pos = []
        self.rope1_len = 0.7419
        self.rope2_len = 0.5527
        self.diag = np.sqrt((self.d*2)**2 + (self.r*2)**2 + (self.r*2)**2)
        self.car1_angle = 0
        self.car2_angle = 0
        self.block_angle = 0
        self.des_left_ten = []
        self.des_right_ten = []
        self.left_car_pos = []
        self.right_car_pos = []
        self.block_all_pos = []
        rospy.Subscriber('/vrep_ros_interface/block_angle', Float32MultiArray, self.block_angle_callback, queue_size=1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/block_pos', Float32MultiArray, self.pos2_callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/car1_pos', Float32MultiArray, self.pos1_callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/car2_pos', Float32MultiArray, self.pos3_callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/tensions', Float32MultiArray, self.ten_callback, queue_size = 1, buff_size=2**8)

        rospy.spin()
        self.plotting()


    def ten_callback(self, msg):
        print('time: ' + str(msg.data[4]))

        self.left_avg_tension.append(np.abs((msg.data[0] + msg.data[1]) / 2))
        self.right_avg_tension.append(np.abs((msg.data[2] + msg.data[3]) / 2))
        self.ten_time.append(msg.data[4])
    
    # car1
    def pos1_callback(self, msg):
        self.car1_pos = [msg.data[0], msg.data[1], msg.data[2]]
        self.left_car_pos.append([msg.data[0], msg.data[1]])
        self.pos_time.append(msg.data[-1])

    # block
    def pos2_callback(self, msg):
        self.block_all_pos.append([msg.data[0], msg.data[1]])
        self.block_pos = [msg.data[0], msg.data[1], msg.data[2]]
        x = np.sqrt((self.diag/2)**2 - (self.block_pos[2])**2)
        self.left_dist = x - self.car1_pos[0]
        self.car1_find_angle()
    
    # car2
    def pos3_callback(self, msg):
        self.car2_pos = [msg.data[0], msg.data[1], msg.data[2]]
        self.right_car_pos.append([msg.data[0], msg.data[1]])
        self.car2_find_angle()
    
    def block_angle_callback(self, msg):
        self.block_angle = msg.data[1]
    
    def car1_find_angle(self):
        c = ((self.d*2)**2 + (self.rope1_len)**2 - (self.left_dist)**2) / (2*(self.d*2)*(self.rope1_len))
        self.car1_angle = np.arccos(c)
        self.calculate()
    
    def car2_find_angle(self):
        g = np.arcsin(self.block_pos[2] / (self.diag + self.rope2_len))
        self.car2_angle = 90 - g
        self.calculate()


    def calculate(self):
        if self.car1_angle != 0 and self.car2_angle != 0 and self.block_angle != 0:
            theta1 = self.car1_angle
            theta2 = self.car2_angle
            theta3 = self.block_angle
            d = self.d
            r = self.r
            ten_mat = np.array([[np.sin(theta1), np.sin(theta2), -np.sin(theta3)],
                                [-np.cos(theta1), -np.cos(theta2), np.cos(theta3)],
                                [-d*np.sin(theta1) + r*np.cos(theta1), d*np.sin(theta2) - r*np.cos(theta2), -r*np.cos(theta3) - d*np.sin(theta3)]])
            des = np.array([[0], 
                            [0], 
                            [self.M*9.81*d*np.cos(theta3)]])
            ten_mat_inv = inv(ten_mat)
            out = ten_mat_inv.dot(des)
            self.des_left_ten.append(out[0][0])
            self.des_right_ten.append(out[1][0])
            print("tension1: " + str(out[0]))
            print("tension2: " + str(out[1]))
            print("f: " + str(out[2]))

    
    def plotting(self):
        # print("hello from plotting")
        # plt.plot(self.left_avg_tension, self.right_avg_tension)
        # plt.xlabel('t1 tension')
        # plt.ylabel('t2 tension')
        # plt.title("t1 vs. t2")
        # plt.show()

        left_x, left_y = [row[0] for row in self.left_car_pos], [row[1] for row in self.left_car_pos]
        block_x, block_y = [row[0] for row in self.block_all_pos], [row[1] for row in self.block_all_pos]
        right_x, right_y = [row[0] for row in self.right_car_pos], [row[1] for row in self.right_car_pos]

        fig, ax = plt.subplots()  # Create a figure and an axes.
        ax.plot(self.ten_time, self.left_avg_tension, color='tab:pink', label='t1')  # Plot some data on the axes.
        ax.plot(self.ten_time, self.right_avg_tension, color='tab:cyan', label='t2')  # Plot more data on the axes...
        ax.plot(self.ten_time, np.abs(self.des_left_ten[:len(self.ten_time)]), color="tab:red", label='desired t1')
        ax.plot(self.ten_time, np.abs(self.des_right_ten[:len(self.ten_time)]), color="tab:blue", label="desired t2")
        plt.legend()
        ax.set_xlabel('Time')  # Add an x-label to the axes.
        color = 'tab:red'
        ax.set_ylabel('Tension', color=color)  # Add a y-label to the axes.
        # ax2 = ax.twinx()
        # ax2.plot(self.pos_time[:len(left_x)], left_x, label='car1 position')
        # ax2.plot(self.pos_time[:len(block_x)], block_x, label='block position')
        # ax2.plot(self.pos_time[:len(right_x)], right_x, label='car2 position')
        # color = 'tab:blue'
        # ax2.set_ylabel('x-axis Position', color=color)
        # plt.title("Tensions, Positions, and Desired Tensions Over Time")  # Add a title to the axes.
        plt.legend()  # Add a legend.
        plt.show()


if __name__ == '__main__':
    rospy.init_node("tensions", anonymous=True)
    t = Tensions()  
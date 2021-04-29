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
from robo_block import Robo_Block

class Tensions():

    def __init__(self, des_angle, des_x_pos):
        self.des_angle = float(des_angle)
        self.des_x_pos = float(des_x_pos)
        self.left_avg_tension = []
        self.right_avg_tension = []
        self.ten_time = []
        self.pos_time = []
        self.d = 0.5/2
        self.r = 0.3/2
        self.left_dist = 0
        self.M = 0.1
        self.I = (1/3)*(self.M)*((2*self.r)**2 + (2*self.d)**2)
        # self.right_dist = 0
        self.car1_pos = []
        self.car2_pos = []
        self.block_pos = []
        self.rope1_len = 0.7419
        self.rope2_len = 0.5527
        self.diag = np.sqrt((self.d*2)**2 + (self.r*2)**2 + (self.r*2)**2)
        self.car1_angle = None
        self.car2_angle = None
        self.block_angle = None
        self.all_block_angle= []
        self.des_left_ten = []
        self.des_right_ten = []
        self.left_car_pos = []
        self.right_car_pos = []
        self.block_all_pos = []
        self.car1_all_pos = []
        self.car2_all_pos = []
        self.block_time = []
        self.robo_block1 = Robo_Block(None, 1, None, None)
        self.robo_block2 = Robo_Block(None, 2, None, None)
        rospy.Subscriber('/vrep_ros_interface/block_angle', Float32MultiArray, self.block_angle_callback, queue_size=1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/block_pos', Float32MultiArray, self.pos2_callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/car_pos1', Float32MultiArray, self.pos1_callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/car_pos2', Float32MultiArray, self.pos3_callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/tensions', Float32MultiArray, self.ten_callback, queue_size = 1, buff_size=2**8)

        while not rospy.is_shutdown():
            # print('i am calling control')
            self.control()
            # print('i have called control')

        self.plotting()


    def ten_callback(self, msg):
        # print('time: ' + str(msg.data[4]))

        self.left_avg_tension.append(np.abs((msg.data[0] + msg.data[1]) / 2))
        self.right_avg_tension.append(np.abs((msg.data[2] + msg.data[3]) / 2))
        self.ten_time.append(msg.data[4])
    
    # car1
    def pos1_callback(self, msg):
        # print('hello from pos1_callback')
        self.car1_pos = [msg.data[0], msg.data[1], msg.data[2]]
        self.car1_all_pos.append(msg.data[0])
        self.left_car_pos.append([msg.data[0], msg.data[1]])
        self.pos_time.append(msg.data[-1])

    # block
    def pos2_callback(self, msg):
        # print('ello from pos2_callback')
        self.block_all_pos.append(msg.data[0])
        self.block_pos = [msg.data[0], msg.data[1], msg.data[2]]
        x = np.sqrt((self.diag/2)**2 - (self.block_pos[2])**2)
        # if self.car1_pos != []:
        self.left_dist = x - self.car1_pos[0]
        self.car1_find_angle()
    
    # car2
    def pos3_callback(self, msg):
        # print('hello from pos3_callback')
        self.car2_pos = [msg.data[0], msg.data[1], msg.data[2]]
        self.car2_all_pos.append(msg.data[0])
        self.right_car_pos.append([msg.data[0], msg.data[1]])
        self.car2_find_angle()
    
    def block_angle_callback(self, msg):
        self.block_angle = msg.data[1]
        self.all_block_angle.append(msg.data[1])
        self.block_time.append(msg.data[-1])
        # print('block angle is: ' + str(self.block_angle))
    
    def car1_find_angle(self):
        c = ((self.d*2)**2 + (self.rope1_len)**2 - (self.left_dist)**2) / (2*(self.d*2)*(self.rope1_len))
        self.car1_angle = np.arccos(c)
        # print('car1 angle is: ' + str(self.car1_angle))
        # self.control()
    
    def car2_find_angle(self):
        g = np.arcsin(self.block_pos[2] / (self.diag + self.rope2_len))
        self.car2_angle = (np.pi/2) - g
        # print('car2 angle is: ' + str(self.car2_angle))
        # self.control()

    # control scheme: find desired wrench to apply to block and calculate necessary tensions to do so and then call robo_block.py for each car to apply the tensions
    def control(self):
        if self.car1_angle and self.car2_angle and self.block_angle:
            # print('hello from control')
            theta1 = self.car1_angle
            theta2 = self.car2_angle
            theta3 = self.des_angle
            torque = self.M*9.81*self.d*np.cos(self.block_angle)
            a_error = theta3 - self.block_angle
            p_error = self.des_x_pos - self.block_pos[0]
            T = 15 # time for acceleration

            self.robo_block1.curr_tension = self.left_avg_tension
            self.robo_block1.curr_pos = self.left_car_pos[-1]

            self.robo_block2.curr_tension = self.right_avg_tension
            self.robo_block2.curr_pos = self.right_car_pos[-1]

            # want torque greater than gravity to rotate block (positive angular acceleration)
            if abs(a_error) > 0.1 or abs(p_error) > 0.05:
                # print('hello the angle error is ' + str(a_error))
                # print('hello the position error is ' + str(p_error))
                des_ang_acc = (4* a_error) / (T**2)
                torque += (self.I*des_ang_acc) # adding torque to rotate block
                des_pos_acc = (4*p_error)/(T**2)
                x_force = self.M*des_pos_acc
                des_wrench = np.array([[x_force], 
                                [0], 
                                [torque]])
                out = self.calculate(theta1, theta2, theta3, des_wrench)
                self.des_left_ten.append(np.abs(out[0][0]))
                self.des_right_ten.append(out[1][0])
                # print(self.des_left_ten)
                # print(des_wrench)
                # print([theta1, theta2])

                # set des_t for robo_block for car1 and car2
                self.robo_block1.des_t = np.abs(out[0][0])
                self.robo_block2.des_t = out[1][0]
                

                # call robo_block control
                # while not self.robo_block1.done and not self.robo_block2.done:
                    # print('hello from control while loop')
                self.robo_block1.curr_tension = self.left_avg_tension
                self.robo_block1.curr_pos = self.left_car_pos[-1]

                self.robo_block2.curr_tension = self.right_avg_tension
                self.robo_block2.curr_pos = self.right_car_pos[-1]
                
                self.robo_block1.control(False)
                self.robo_block2.control(False)
                # print('end of control while loop. done = ' + str(self.robo_block1.done))
                # print('end of control while loop. done = ' + str(self.robo_block2.done))


            # just resist gravity and stop rotating if within error tolerance
            else:
                print('hello i am just resisting gravity')
                torque = self.M*9.81*self.d*np.cos(self.block_angle)
                theta3 = self.block_angle
                des_wrench = np.array([[0], 
                                [0], 
                                [torque]])
                out = self.calculate(theta1, theta2, theta3, des_wrench)
                self.des_left_ten.append(np.abs(out[0][0]))
                self.des_right_ten.append(out[1][0])
                # print(self.des_left_ten)
                # print(des_wrench)
                # print([theta1, theta2])

                # set des_t for robo_block for car1 and car2
                self.robo_block1.des_t = np.abs(out[0][0])
                self.robo_block2.des_t = out[1][0]
                # print('des tensions are: ' + str(out))

                # while not rospy.is_shutdown():
                self.robo_block1.curr_tension = self.left_avg_tension
                self.robo_block1.curr_pos = self.left_car_pos[-1]

                self.robo_block2.curr_tension = self.right_avg_tension
                self.robo_block2.curr_pos = self.right_car_pos[-1]
                
                self.robo_block1.control(True)
                self.robo_block2.control(True)

                

    
    # calculate necessary tensions
    def calculate(self, theta1, theta2, theta3, des_wrench):
        d = self.d
        r = self.r
        ten_mat = np.array([[np.sin(theta1), np.sin(theta2), -np.sin(theta3)],
                            [-np.cos(theta1), -np.cos(theta2), np.cos(theta3)],
                            [-d*np.sin(theta1) + r*np.cos(theta1), d*np.sin(theta2) - r*np.cos(theta2), -r*np.cos(theta3) - d*np.sin(theta3)]])
        ten_mat_inv = inv(ten_mat)
        out = ten_mat_inv.dot(des_wrench)
        return out

    
    def plotting(self):
        # print("hello from plotting")
        # plt.plot(self.left_avg_tension, self.right_avg_tension)
        # plt.xlabel('t1 tension')
        # plt.ylabel('t2 tension')
        # plt.title("t1 vs. t2")
        # plt.show()
        MEDIUM_SIZE = 12
        plt.rc('font', size=MEDIUM_SIZE)          # controls default text sizes
        plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
        plt.rc('xtick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
        plt.rc('ytick', labelsize=MEDIUM_SIZE)    # fontsize of the tick labels
        plt.rc('legend', fontsize=MEDIUM_SIZE)    # legend fontsize
        plt.rc('figure', titlesize=MEDIUM_SIZE)  # fontsize of the figure title

        #fig, ax = plt.subplots()  # Create a figure and an axes.
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(self.ten_time, np.abs(self.left_avg_tension[:len(self.ten_time)]), color='tab:pink', label='current t1')  # Plot some data on the axes.
        plt.plot(self.ten_time, np.abs(self.right_avg_tension[:len(self.ten_time)]), color='tab:cyan', label='current t2')  # Plot more data on the axes...
        plt.xlabel('Time (s)') 
        plt.ylabel('Tension (N)')
        plt.title('Current and Desired Tensions')
        plt.legend()
        plt.subplot(2, 1, 2)
        plt.plot(self.ten_time, np.abs(self.des_left_ten[:len(self.ten_time)]), color="tab:red", label='desired t1')
        plt.plot(self.ten_time, np.abs(self.des_right_ten[:len(self.ten_time)]), color="tab:blue", label="desired t2")
        plt.xlabel('Time (s)')  # Add an x-label to the axes.
        # color = 'tab:red'
        plt.ylabel('Tension (N)')  # Add a y-label to the axes.
        plt.legend()
        plt.show()

        # fig2, ax2 = plt.subplots()
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(self.block_time, self.all_block_angle[:len(self.block_time)], label='block angle')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (rad)')
        plt.title('Block Angle (Desired: ' + str(self.des_angle) + ' rad), Block Position (Desired: ' + str(self.des_x_pos) + 'm)')
        plt.legend()  # Add a legend.
        # plt.show()

        # fig3, ax3 = plt.subplots()
        plt.subplot(2, 1, 2)
        plt.plot(self.block_time, self.block_all_pos[:len(self.block_time)], label='block position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.legend()
        plt.show()

        plt.figure()
        plt.plot(self.block_time, self.car1_all_pos[:len(self.block_time)], label='robot1 position')
        plt.plot(self.block_time, self.car2_all_pos[:len(self.block_time)], label='robot2 position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Robot Positions')
        plt.legend()
        plt.show()


if __name__ == '__main__':
    des_a = raw_input("Please enter desired block angle (rad): ")
    des_p = raw_input("Please enter desired block x position (m): ")
    rospy.init_node("tensions", anonymous=True)
    t = Tensions(des_a, des_p)

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

class Pulling():
    def __init__(self):
        self.left_avg_tension = []
        self.right_avg_tension = []
        self.left_car_pos = []
        self.right_car_pos = []
        self.block_pos = []
        self.ten_time= []
        self.pos_time = []
        rospy.Subscriber('/vrep_ros_interface/tensions', Float32MultiArray, self.ten_callback, queue_size = 1, buff_size=2**8)
        # rospy.Subscriber('/vrep_ros_interface/torques', Float32MultiArray, self.callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/car1_pos', Float32MultiArray, self.pos1_callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/block_pos', Float32MultiArray, self.pos2_callback, queue_size = 1, buff_size=2**8)
        rospy.Subscriber('/vrep_ros_interface/car2_pos', Float32MultiArray, self.pos3_callback, queue_size = 1, buff_size=2**8)

        self.carPub = rospy.Publisher('/vrep_ros_interface/car_vel', Float32MultiArray, queue_size=10)

        
        # while not rospy.is_shutdown():
        rospy.spin()
        self.plotting()

    def ten_callback(self, msg):
        print('time: ' + str(msg.data[4]))
        # print('tension1: ' + str(msg.data[0]))
        # print('tension2: ' + str(msg.data[1]))
        # print('tension3: ' + str(msg.data[2]))
        # print('tension4: ' + str(msg.data[3]) + '\n')

        self.left_avg_tension.append((msg.data[0] + msg.data[1]) / 2)
        self.right_avg_tension.append((msg.data[2] + msg.data[3]) / 2)
        self.ten_time.append(msg.data[4])
    
    # car1 (left) pos
    def pos1_callback(self, msg):
        self.left_car_pos.append([msg.data[0], msg.data[1]])
        self.pos_time.append(msg.data[-1])
        # print(str(self.left_car_pos) + '\n')
    
    # block pos
    def pos2_callback(self, msg):
        self.block_pos.append([msg.data[0], msg.data[1]])
    
    # car2 (right) pos
    def pos3_callback(self, msg):
        self.right_car_pos.append([msg.data[0], msg.data[1]])



    # Need to fix
    def lowPassFilter(self, data):
        order = 5
        sampling_freq = 30
        cutoff_freq = 2
        normalized_cutoff_freq = 2 * cutoff_freq / sampling_freq
        numerator_coeffs, denominator_coeffs = scipy.signal.butter(order, normalized_cutoff_freq)
        filtered_signal = scipy.signal.lfilter(numerator_coeffs, denominator_coeffs, data)
        return filtered_signal
    
    def plotting(self):
        # print("hello from plotting")
        # plt.plot(self.left_avg_tension, self.right_avg_tension)
        # plt.xlabel('t1 tension')
        # plt.ylabel('t2 tension')
        # plt.title("t1 vs. t2")
        # plt.show()

        left_x, left_y = [row[0] for row in self.left_car_pos], [row[1] for row in self.left_car_pos]
        block_x, block_y = [row[0] for row in self.block_pos], [row[1] for row in self.block_pos]
        right_x, right_y = [row[0] for row in self.right_car_pos], [row[1] for row in self.right_car_pos]

        fig, ax = plt.subplots()  # Create a figure and an axes.
        ax.plot(self.ten_time, self.left_avg_tension, color='tab:pink', label='t1')  # Plot some data on the axes.
        ax.plot(self.ten_time, self.right_avg_tension, color='tab:cyan', label='t2')  # Plot more data on the axes...
        plt.legend()
        ax.set_xlabel('Time')  # Add an x-label to the axes.
        color = 'tab:red'
        ax.set_ylabel('Tension', color=color)  # Add a y-label to the axes.
        ax2 = ax.twinx()
        ax2.plot(self.pos_time, left_x, label='car1 position')
        ax2.plot(self.pos_time, block_x, label='block position')
        ax2.plot(self.pos_time, right_x, label='car2 position')
        color = 'tab:blue'
        ax2.set_ylabel('x-axis Position', color=color)
        plt.title("Tensions and Positions Over Time")  # Add a title to the axes.
        plt.legend()  # Add a legend.
        plt.show()

        # pos_fig = plt.figure()
        # pos_ax = ax = pos_fig.gca(projection='3d') #plt.axes(projection="3d")
        # pos_ax.plot(self.pos_time, left_x, left_y, label='car1 position')
        # pos_ax.plot(self.pos_time, block_x, block_y, label='block positon')
        # pos_ax.plot(self.pos_time, right_x, right_y, label='car2 positon')
        # plt.title('Positions over Time')
        # pos_ax.set_xlabel('Time (s)')
        # pos_ax.set_ylabel('x-axis Position')
        # pos_ax.set_zlabel('y-axis Position')
        # plt.legend()
        # plt.show()

        # plt.plot(self.pos_time, left_x, label='car1 position')
        # plt.plot(self.pos_time, block_x, label='block position')
        # plt.plot(self.pos_time, right_x, label='car2 position')
        # plt.xlabel('Time (s)')
        # plt.ylabel('x-axis Position')
        # plt.legend()
        # plt.title('Positions Over Time')
        # plt.show()


if __name__ == '__main__':
    rospy.init_node("pulling", anonymous=True)
    p = Pulling()



# fs=simGetObjectHandle('Forcesensor')

# r,force,torque=simReadForceSensor(fs)

# print (r)
# print ('forces are '..force[1]..', '..force[2]..', '..force[3])
# print ('torques are '..torque[1]..', '..torque[2]..', '..torque[3])
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

class Controller():

    def __init__(self, des_pos, curr_pos, car):
        self.des_pos = des_pos
        self.curr_pos = curr_pos
        self.Kx = 30.0
        self.Ky = 20.0
        self.car = car
        # self.all_curr_vel = []
        # self.time = []
        self.all_pos = []
        self.vel_pub = rospy.Publisher('/vrep_ros_interface/car_vel' + str(car), Float32MultiArray, queue_size=10)
        # rospy.Subscriber('/vrep_ros_interface/car_curr_vel', Float32MultiArray, self.vel_callback, queue_size=1, buff_size=2**8)
        # rospy.Subscriber('/vrep_ros_interface/car_pos', Float32MultiArray, self.pos_callback, queue_size=1, buff_size=2**8)
        # rospy.spin()
        # self.plotting()

    def set_velocity(self):
        # print('hello from set_velocity')
        # self.all_curr_vel.append([msg.data[0], msg.data[1]])
        # self.time.append(msg.data[-1])
        curr_y_pos = self.curr_pos[1]
        des_y_pos = self.curr_pos[1]
        curr_x_pos = self.curr_pos[0]
        des_x_pos = self.des_pos
        # print(des_x_pos)
        # print(curr_x_pos)

        error_y = des_y_pos - curr_y_pos
        error_x = des_x_pos - curr_x_pos

        vel = Float32MultiArray()
        vel.data = [self.Kx*error_x, self.Ky*error_y, 0]
        self.vel_pub.publish(vel)
        # print('set velocity to: ' + str(vel.data))
    
    # def pos_callback(self, msg):
    #     self.all_pos.append(msg.data[1])
    
    def plotting(self):
        # print("hello from plotting")
        # print(self.all_curr_vel)
        fig, ax = plt.subplots()  # Create a figure and an axes.
        ax.plot(self.time, np.array(self.all_curr_vel)[:,0], color='tab:pink', label='x vel')  # Plot some data on the axes.
        ax.plot(self.time, np.array(self.all_curr_vel)[:,1], color='tab:cyan', label='y vel')  # Plot more data on the axes...
        plt.legend()
        ax.set_xlabel('Time (s)')  # Add an x-label to the axes.
        color = 'tab:red'
        ax.set_ylabel('Velocity (m/s)', color=color)
        ax.set_ylim([-0.1, 0.1])

        ax2 = ax.twinx()
        ax2.plot(self.time, self.all_pos[:len(self.time)], label='car y position')
        color = 'tab:blue'
        ax2.set_ylabel('y-axis Position (m)', color=color)
        ax2.set_ylim([-0.5, 0])

        plt.title('Car x, y Velocity and y Position with P Controller')

        plt.legend()
        plt.show()


# if __name__ == '__main__':
#     rospy.init_node("controller", anonymous=True)
#     c = Controller() 

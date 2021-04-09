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

    def __init__(self):
        self.K = 10.0
        self.all_curr_vel = []
        self.time = []
        self.vel_pub = rospy.Publisher('/vrep_ros_interface/car_vel', Float32MultiArray, queue_size=10)
        rospy.Subscriber('/vrep_ros_interface/car_curr_vel', Float32MultiArray, self.vel_callback, queue_size=1, buff_size=2**8)
        rospy.spin()
        self.plotting()


    def vel_callback(self, msg):
        print('hello from vel_callback')
        self.all_curr_vel.append([msg.data[0], msg.data[1]])
        self.time.append(msg.data[-1])
        curr_y_vel = msg.data[1]
        des_y_vel = 0
        error_y = des_y_vel - curr_y_vel
        vel = Float32MultiArray()
        vel.data = [msg.data[0], self.K*error_y, msg.data[2]]
        self.vel_pub.publish(vel)
    
    def plotting(self):
        # print("hello from plotting")
        print(self.all_curr_vel)
        fig, ax = plt.subplots()  # Create a figure and an axes.
        ax.plot(self.time, np.array(self.all_curr_vel)[:,0], color='tab:pink', label='x vel')  # Plot some data on the axes.
        ax.plot(self.time, np.array(self.all_curr_vel)[:,1], color='tab:cyan', label='y vel')  # Plot more data on the axes...
        plt.legend()
        ax.set_xlabel('Time')  # Add an x-label to the axes.
        color = 'tab:red'
        ax.set_ylabel('Velocity', color=color)
        plt.legend()
        plt.show()


if __name__ == '__main__':
    rospy.init_node("controller", anonymous=True)
    c = Controller() 

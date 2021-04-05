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

class Pulling():
    def __init__(self):
        self.right_avg_tension = []
        self.time= []
        rospy.Subscriber('/vrep_ros_interface/tensions', Float32MultiArray, self.callback, queue_size = 1, buff_size=2**8)
        # rospy.Subscriber('/vrep_ros_interface/car1_pos', Float32MultiArray, self.callback, queue_size = 1, buff_size=2**8)
        # rospy.Subscriber('/vrep_ros_interface/block_pos', Float32MultiArray, self.callback, queue_size = 1, buff_size=2**8)
        # rospy.Subscriber('/vrep_ros_interface/car2_pos', Float32MultiArray, self.callback, queue_size = 1, buff_size=2**8)
        # # while not rospy.is_shutdown():
        rospy.spin()
        self.plotting()

    def callback(self, msg):
        print('time: ' + str(msg.data[-1]))
        print('tension1: ' + str(msg.data[0]))
        print('tension2: ' + str(msg.data[1]))

        self.right_avg_tension.append((msg.data[0] + msg.data[1]) / 2)
        self.time.append(msg.data[-1])

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
        # plt.plot(self.time, self.right_avg_tension, label='t2') 
        # filtered_signal = self.lowPassFilter(self.right_avg_tension)
        plt.plot(self.time, self.right_avg_tension, label='t2') 
        plt.xlabel('Time')  # Add an x-label to the axes.
        plt.ylabel('Tension')  # Add a y-label to the axes.
        engine = raw_input("sim engine: ")
        plt.title("Tensions over Time (" + engine + ")")  # Add a title to the axes.
        plt.legend()  # Add a legend.
        plt.show()


if __name__ == '__main__':
    rospy.init_node("pulling", anonymous=True)
    p = Pulling()



# fs=simGetObjectHandle('Forcesensor')

# r,force,torque=simReadForceSensor(fs)

# print (r)
# print ('forces are '..force[1]..', '..force[2]..', '..force[3])
# print ('torques are '..torque[1]..', '..torque[2]..', '..torque[3])
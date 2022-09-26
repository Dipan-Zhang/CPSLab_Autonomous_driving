#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Duration
from std_msgs.msg import Float32


def plot_curve_angle(msg):
    global counter
    global angledata
    if (counter% 20 == 0):
        angledata.append(msg.data)
        plt.plot(angledata,'k')
        plt.title('turning_angle')
        plt.draw()
        plt.pause(0.01)

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.Subscriber("/turning_angle", Float32, plot_curve_angle)
    angledata = []
    rospy.init_node("simdata")

    plt.ion()
    rospy.spin()

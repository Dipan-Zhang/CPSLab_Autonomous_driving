#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Duration
from std_msgs.msg import Float32


def plot_curve_error(msg):
    global counter
    global errordata
    if (counter% 20 == 0):
        errordata.append(msg.data)
        plt.plot(errordata,'k')
        plt.title('errordata')
        plt.draw()
        plt.pause(0.01)

    counter += 1

if __name__ == '__main__':
    counter = 0

    rospy.Subscriber("/error", Float32, plot_curve_error)
    errordata = []
    rospy.init_node("errordata")

    plt.ion()
    rospy.spin()

#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import threading

from helpers.plot import Plot2

plot = Plot2()
speed = None
height = None


def droneheightCb(msg):
    global height

    height = msg.data

    if height is not None:
        rospy.loginfo(height)
        plot.update(height)


def dronespeedCb(msg):
    global speed

    speed = msg.data

    if speed is not None:
        rospy.loginfo(speed)
        plot.update(speed)


if __name__ == '__main__':
    rospy.init_node("PID_plot", anonymous=False)
    rospy.Subscriber("/anafi/horizontal_speed", Float32, dronespeedCb)
    # rospy.Subscriber("/anafi/height", Float32, droneheightCb)

    plot_title = "PID"
    pid_setpoint = 0.5

    threading.Thread(target=plot.plotter, args=(1, plot_title, pid_setpoint,)).start()

    rospy.spin()

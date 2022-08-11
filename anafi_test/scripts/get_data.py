#!/usr/bin/env python3

from distutils.command.install_egg_info import safe_version
import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int16
from tf import transformations
import csv
import time


def read_data(msg):
    global a
    global saver
    global r_list, p_list, y_list

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    print("[ROS INFO] x:{:.2f}, y:{:.2f}, z:{:.2f}".format(x, y, z))

    # orientation of the AR tag
    roll, pitch, yaw = transformations.euler_from_quaternion(
        [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    print("[ROS INFO] roll:{:.3f}, pitch:{:.3f}, yaw:{:.3f}".format(
        roll, pitch, yaw))

    roll_deg = roll * 180/math.pi
    pitch_deg = pitch * 180/math.pi
    yaw_deg = yaw * 180/math.pi

    if saver:
        r_list.append(roll_deg)
        p_list.append(pitch_deg)
        y_list.append(yaw_deg)
    else:
        r_list = []
        p_list = []
        y_list = []

    a = np.asarray([r_list, p_list, y_list])

    # print("[ROS INFO] roll:{:.3f}, pitch:{:.3f}, yaw:{:.3f}".format(
    #     roll_deg, pitch_deg, yaw_deg))


def save_bool(msg):
    global saver
    global c
    global timer_bool_act

    saver = msg.data
    if saver:
        timer_bool_act = True

    if not saver and timer_bool_act == True:
        c = time.time()

        _dir = "/home/anafi/anafi_ws/src/anafi_ros/anafi_test/results/"
        f_name = _dir + "test_" + str(c) + ".csv"
        with open(f_name, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['roll', 'pitch', 'yaw'])
            for i in range(len(r_list)):
                writer.writerow([r_list[i], p_list[i], y_list[i]])
        f.close()

        timer_bool_act = False

    # print("[ROS INFO] save:", saver)


rospy.init_node("get_data_node")
rospy.loginfo('Getting Anafi Pose')
rospy.Subscriber("/vrpn_client_node/Anafi/pose", PoseStamped, read_data)
rospy.Subscriber("/anafi/save_data", Int16, save_bool)
r_list = []
p_list = []
y_list = []
saver = 0
c = 0
timer_bool_act = False

if __name__ == '__main__':
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('...exiting due to keyboard interrupt')

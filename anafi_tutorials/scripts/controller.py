#!/usr/bin/env python3

import olympe
import olympe_deps as od
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged, SpeedChanged
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from std_msgs.msg import Int16, Time, Header, Float32, UInt8
from geometry_msgs.msg import Twist, PointStamped, Vector3Stamped, QuaternionStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

import time
import numpy as np
import pygame
import rospy
import sys

largest = None   # define it as 0


class Anafi:

    def __init__(self):
        # Publishers
        self.pub_time = rospy.Publisher(
            "/anafi/time", Time, queue_size=1)
        self.pub_attitude = rospy.Publisher(
            "/anafi/attitude", QuaternionStamped, queue_size=1)
        self.pub_location = rospy.Publisher(
            "/anafi/location", PointStamped, queue_size=1)
        self.pub_height = rospy.Publisher(
            "/anafi/height", Float32, queue_size=1)
        self.pub_speed = rospy.Publisher(
            "/anafi/speed", Vector3Stamped, queue_size=1)
        self.pub_air_speed = rospy.Publisher(
            "/anafi/air_speed", Float32, queue_size=1)
        self.pub_battery = rospy.Publisher(
            "/anafi/battery", UInt8, queue_size=1)
        self.pub_pose = rospy.Publisher(
            "/anafi/pose", PoseStamped, queue_size=1)
        self.pub_odometry = rospy.Publisher(
            "/anafi/odometry", Odometry, queue_size=1)
        self.pub_image = rospy.Publisher("/anafi/image", Image, queue_size=1)
        self.pub_horizontalspeed = rospy.Publisher(
            "/anafi/horizontal_speed", Float32, queue_size=1)

        # # Subscribers
        # rospy.Subscriber("/anafi/takeoff", Int16, self._takeoff)
        # rospy.Subscriber("/anafi/land", Int16, self._land)
        # rospy.Subscriber("/anafi/cmd_vel", Twist, self._cmdVel)
        # rospy.Subscriber("/anafi/height", Float32, self._altitude)

        # subscribe to speed
        # rospy.Subscriber(
        #     "/anafi/speed", Vector3Stamped, self.compute_PID)

        DRONE_IP = '10.202.0.1'  # SIM
        # DRONE_IP = "192.168.42.1"  # REAL
        self.drone = olympe.Drone(
            DRONE_IP, drone_type=od.ARSDK_DEVICE_TYPE_ANAFI4K)

        # params

    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnection()

    def keyboard_control(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # quit safely by landing drone
                self.drone.piloting_pcmd(0, 0, 0, 0, 0.0)
                time.sleep(1)
                self.drone(Landing() >> FlyingStateChanged(
                    state="landing", _timeout=5)).wait()
                pygame.display.quit()
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYUP:
                self.vxx = 0
                self.vyy = 0
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                   # non-blocking PCMD (roll, pitch, yaw, thrust, piloting_time)
                    self.drone.piloting_pcmd(0, 0, 0, 0, 0.0)
                    time.sleep(1)
                elif event.key == pygame.K_t:
                    self.drone(TakeOff() >> FlyingStateChanged(
                        state="hovering", _timeout=5)).wait()
                elif event.key == pygame.K_SPACE:
                    self.drone(Landing() >> FlyingStateChanged(
                        state="landing", _timeout=5)).wait()

                elif event.key == pygame.K_w:
                    self.vxx = 20
                elif event.key == pygame.K_s:
                    self.vxx = -20
                elif event.key == pygame.K_a:
                    self.vyy = -20
                elif event.key == pygame.K_d:
                    self.vyy = 20

        # non-blocking PCMD (roll, pitch, yaw, thrust, piloting_time)
        self.drone.piloting_pcmd(self.vyy, self.vxx, 0, 0, 0.2)

    def start(self):
        rospy.init_node("controller_node")
        # Connect the the drone
        self.drone.connection()

        # start piloting thread
        self.drone.start_piloting()

        self.vxx = 0
        self.vyy = 0

        rate = rospy.Rate(100)  # loop rate 100Hz ~ 0.01s
        try:
            while not rospy.is_shutdown():
                self.keyboard_control()
                # display pygame screen for keyboard control
                pygame.display.flip()
                rate.sleep()

        except:
            rospy.logfatal("Controller failed")


if __name__ == "__main__":
    pygame.init()
    W, H = 160, 120
    screen = pygame.display.set_mode((W, H))
    drone = Anafi()
    drone.start()

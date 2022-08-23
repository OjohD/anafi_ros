#!/usr/bin/env python3
from sys import flags
import time
import olympe
import rospy

import numpy as np
import math
import cv2
import queue
import threading
from cv_bridge import CvBridge

from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.PilotingSettingsState import MaxTiltChanged
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AttitudeChanged, SpeedChanged, AltitudeChanged, ReturnHomeBatteryCapacity
from olympe.messages.battery import voltage
from std_msgs.msg import Int16, Time, Header, Float32, UInt8
from geometry_msgs.msg import Twist, PointStamped, Vector3Stamped, QuaternionStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from anafi_tutorials.msg import piloting_CMD, drone_rpy

from scipy.spatial.transform import Rotation as R


class anafi():
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
        self.pub_camera = rospy.Publisher(
            "/anafi/image", Image, queue_size=1)
        self.pub_horizontalspeed = rospy.Publisher(
            "/anafi/horizontal_speed", Float32, queue_size=1)
        self.pub_rpy = rospy.Publisher(
            "/anafi/rpy", drone_rpy, queue_size=1)
        # self.pub_odometry = rospy.Publisher(         pass
        self.bridge = CvBridge()
        self.rpy_msg = drone_rpy()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()

        # self.status = False
        self.connected = False

        # DRONE_IP = '10.202.0.1'  # SIM
        DRONE_IP = "192.168.42.1"  # REAL
        self.drone = olympe.Drone(DRONE_IP)
        self.connection = self.drone.connect()
        if getattr(self.connection, 'OK') == True:
            print('connected')
            self.connected = True
            # # start piloting thread
            # self.drone.start_piloting()
            # Setup the callback functions to do some live video processing
            self.drone.set_streaming_callbacks(
                raw_cb=self.yuv_frame_cb)
            # flush_raw_cb=self.flush_cb)
            self.drone.start_video_streaming()

            # set max tilt
            self.drone(MaxTilt(5)).wait().success()
        else:
            print('No connection')

        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        time.sleep(3)

        self.previousTime = time.time()
        self.takeoffTime = 0

        self._meta_data = 0

        self.manual_control = False
        self.trigger_height = None
        self.emergency_land = False
        self.height_offset = 3.5  # SIM = 0    Real = 3.5
        self.target_altitude = 1.5  # meters

        self.pressing = 0
        self.is_pressed = 0

        # Subscribers
        rospy.Subscriber("/anafi/takeoff", Int16, self._takeoff)
        rospy.Subscriber("/anafi/land", Int16, self._land)
        rospy.Subscriber("/anafi/flight_control", piloting_CMD, self._cmdVel)
        rospy.Subscriber("/anafi/height", Float32, self._altitude)

    # def clamp(self, x,  in_min,  in_max,  out_min,  out_max):
    #     if x > in_max:
    #         x = in_max

    #     if x < in_min:
    #         x = in_min

    #     res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    #     return np.int8(res)

    def yuv_frame_cb(self, yuv_frame):
        # source https://github.com/hildebrandt-carl/MixedRealityTesting/blob/master/monocular_avoidance_ws/src/drone_controller/src/main_drone.py
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        # Send the image on ROS
        image_message = self.bridge.cv2_to_imgmsg(cv2frame, encoding="bgr8")
        self.pub_camera.publish(image_message)

    def log_changes(self):

        # drone height
        drone_height = self.drone.get_state(AltitudeChanged)
        self.pub_height.publish(drone_height['altitude'])

        drone_rpy = self.drone.get_state(AttitudeChanged)
        speed_dict = self.drone.get_state(SpeedChanged)
        # speed_dict['speedX']

        # horizontal Speed (Magnitude) (m/s)
        drone_speed = math.sqrt(
            (speed_dict['speedX'])**2 + (speed_dict['speedY'])**2)
        self.pub_horizontalspeed.publish(drone_speed)

        # drone max tilt
        # self.drone(MaxTilt(10)).wait().success()
        current_max_tilt = self.drone.get_state(MaxTiltChanged)["current"]

        # do not want to flood terminal
        if time.time()-self.previousTime < 10:
            rospy.loginfo("current max_tilt--------:"+str(current_max_tilt))

        self.rpy_msg.roll  = drone_rpy['roll'] * 180/math.pi
        self.rpy_msg.pitch = drone_rpy['pitch'] * 180/math.pi
        self.rpy_msg.yaw = drone_rpy['yaw'] * 180/math.pi


        self.pub_rpy.publish(self.rpy_msg)

        rospy.loginfo("-----------RPY-----------")
        rospy.loginfo("r=%.2f, p=%.2f, y=%.2f", self.rpy_msg.roll, self.rpy_msg.pitch, self.rpy_msg.yaw)

    def _takeoff(self, msg):
        if msg.data == 1:
            self.trigger_height = True
            self.takeoffTime = time.time()
            self.emergency_land = False

            # start piloting thread
            self.drone.start_piloting()
            # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.TakeOff
            self.drone(TakeOff() >> FlyingStateChanged(
                state="hovering", _timeout=10)).wait()

    def _land(self, msg):
        if msg.data == 2:
            self.emergency_land = True
            self.target_altitude = 1.5
            self.trigger_height = None
            # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.Landing
            self.drone(Landing()).wait()
            print("Landed")

    # PID conroller
    # Consider making a thread for this

    def _cmdVel(self, msg):

        # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.PCMD

        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        thrust = msg.thrust

        # // If roll or pitch value are non-zero, enabel roll/pitch flag
        flag = not ((abs(roll) < 0.001) and (abs(pitch) < 0.001))

        # non-blocking PCMD (roll, pitch, yaw, thrust, piloting_time)
        self.drone.piloting_pcmd(roll, pitch, yaw, thrust, 0.2)
        # self.drone(PCMD(flag, roll, pitch, yaw, thrust, 0))

    def press(self):
        if self.is_pressed != self.pressing:
            self.manual_control = False
            self.is_pressed += 1
            rospy.loginfo("manual_control = False")
            rospy.loginfo("is_pressed: %d, pressing: %d",
                          self.is_pressed, self.pressing)

        else:
            self.manual_control = True
            self.pressing = 0
            self.is_pressed = 0
            if self.manual_control and self.trigger_height is False:
                rospy.loginfo("Manual control Set. PID Altitude Auto tune")


if __name__ == '__main__':
    rospy.init_node("anafi_node")
    rate = rospy.Rate(10)
    app = anafi()
    try:
        while not rospy.is_shutdown():
            app.log_changes()
            rate.sleep()
    except KeyboardInterrupt:
        pass

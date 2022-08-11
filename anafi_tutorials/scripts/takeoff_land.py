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

from olympe.messages.ardrone3.Piloting import TakeOff, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AttitudeChanged, SpeedChanged, AltitudeChanged, ReturnHomeBatteryCapacity
from olympe.messages.battery import voltage
from std_msgs.msg import Int16, Time, Header, Float32, UInt8
from geometry_msgs.msg import Twist, PointStamped, Vector3Stamped, QuaternionStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from anafi_tutorials.msg import piloting_CMD

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
        # self.pub_odometry = rospy.Publisher(         pass
        self.bridge = CvBridge()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()

        self.status = False
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
        else:
            print('No connection')

        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        time.sleep(3)

        # init PID params
        self.previousTime = time.time()
        self.takeoffTime = 0

        self.manualTime = 0
        self.cumError_pitch = 0
        self.cumError_roll = 0
        self.cumError_z = 0
        self.lastError_pitch = 0
        self.lastError_roll = 0
        self.PID_pitch = 0
        self.PID_roll = 0

        self.roll = 0
        self.pitch = 0
        self.P_gaz = 0

        self.kp = 1

        self.target_pitch = 0
        self.target_roll = 0
        self.thrust = 0

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

    def clamp(self, x,  in_min,  in_max,  out_min,  out_max):
        if x > in_max:
            x = in_max

        if x < in_min:
            x = in_min

        res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        return np.int8(res)

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

    # def flush_cb(self):
    #     # source https://github.com/andriyukr/olympe_bridge
    #     with self.flush_queue_lock:
    #         while not self.frame_queue.empty():
    #             self.frame_queue.get_nowait().unref()
    #     return True

    # def yuv_callback(self, yuv_frame):
    #     # source https://github.com/andriyukr/olympe_bridge
    #     # Use OpenCV to convert the yuv frame to RGB
    #     # the VideoFrame.info() dictionary contains some useful information such as the video resolution
    #     info = yuv_frame.info()
    #     rospy.logdebug_throttle(10, "yuv_frame.info = " + str(info))
    #     cv2_cvt_color_flag = {
    #         olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
    #         olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
    #     }[info["yuv"]["format"]]  # convert pdraw YUV flag to OpenCV YUV flag
    #     cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

    #     # Publish image
    #     msg_image = self.bridge.cv2_to_imgmsg(cv2frame, "bgr8")
    #     self.pub_image.publish(msg_image)

    #     # yuv_frame.vmeta() returns a dictionary that contains additional metadata from the drone (GPS coordinates, battery percentage, ...)
    #     metadata = yuv_frame.vmeta()
    #     rospy.logdebug_throttle(10, "yuv_frame.vmeta = " + str(metadata))

        # if metadata[1] != None:
        #     header = Header()
        #     header.stamp = rospy.Time.now()
        #     header.frame_id = '/body'

        #     # timestamp [millisec]
        #     frame_timestamp = metadata[1]['frame_timestamp']
        #     msg_time = Time()
        #     # secs = int(frame_timestamp//1e6), nsecs = int(frame_timestamp%1e6*1e3)
        #     msg_time.data = frame_timestamp
        #     self.pub_time.publish(msg_time)

        #     drone_quat = metadata[1]['drone_quat']  # attitude
        #     msg_attitude = QuaternionStamped()
        #     msg_attitude.header = header
        #     msg_attitude.quaternion = Quaternion(
        #         drone_quat['x'], -drone_quat['y'], -drone_quat['z'], drone_quat['w'])
        #     self.pub_attitude.publish(msg_attitude)

        #     # GPS location [500.0=not available] (decimal deg)
        #     location = metadata[1]['location']
        #     msg_location = PointStamped()
        #     if location != {}:
        #         msg_location.header = header
        #         msg_location.header.frame_id = '/world'
        #         msg_location.point.x = location['latitude']
        #         msg_location.point.y = location['longitude']
        #         msg_location.point.z = location['altitude']
        #         self.pub_location.publish(msg_location)

        #     # barometer (m)
        #     ground_distance = metadata[1]['ground_distance']
        #     self.pub_height.publish(ground_distance)

        #     speed = metadata[1]['speed']  # opticalflow speed (m/s)
        #     msg_speed = Vector3Stamped()
        #     msg_speed.header = header
        #     msg_speed.header.frame_id = '/world'
        #     msg_speed.vector.x = speed['north']
        #     msg_speed.vector.y = -speed['east']
        #     msg_speed.vector.z = -speed['down']
        #     self.pub_speed.publish(msg_speed)

        #     # horizontal Speed (Magnitude) (m/s)
        #     drone_speed = math.sqrt(
        #         (msg_speed.vector.x)**2 + (msg_speed.vector.y)**2)
        #     self.pub_horizontalspeed.publish(drone_speed)

        #     # air speed [-1=no data, > 0] (m/s)
        #     air_speed = metadata[1]['air_speed']
        #     self.pub_air_speed.publish(air_speed)

        #     # [0=empty, 100=full]
        #     battery_percentage = metadata[1]['battery_percentage']
        #     self.pub_battery.publish(battery_percentage)

        #     msg_pose = PoseStamped()
        #     msg_pose.header = header
        #     msg_pose.pose.position = msg_location.point
        #     msg_pose.pose.position.z = ground_distance
        #     msg_pose.pose.orientation = msg_attitude.quaternion
        #     self.pub_pose.publish(msg_pose)

        #     Rot = R.from_quat(
        #         [drone_quat['x'], -drone_quat['y'], -drone_quat['z'], drone_quat['w']])
        #     drone_rpy = Rot.as_euler('xyz')

        #     msg_odometry = Odometry()
        #     msg_odometry.header = header
        #     msg_odometry.child_frame_id = '/body'
        #     msg_odometry.pose.pose = msg_pose.pose
        #     msg_odometry.twist.twist.linear.x = math.cos(
        #         drone_rpy[2])*msg_speed.vector.x + math.sin(drone_rpy[2])*msg_speed.vector.y
        #     msg_odometry.twist.twist.linear.y = - \
        #         math.sin(drone_rpy[2])*msg_speed.vector.x + \
        #         math.cos(drone_rpy[2])*msg_speed.vector.y
        #     msg_odometry.twist.twist.linear.z = msg_speed.vector.z
        #     self.pub_odometry.publish(msg_odometry)

        #     # log battery percentage
        #     if battery_percentage >= 30:
        #         if battery_percentage % 10 == 0:
        #             rospy.loginfo_throttle(
        #                 100, "Battery level: " + str(battery_percentage) + "%")
        #     else:
        #         if battery_percentage >= 20:
        #             rospy.logwarn_throttle(
        #                 10, "Low battery: " + str(battery_percentage) + "%")
        #         else:
        #             if battery_percentage >= 10:
        #                 rospy.logerr_throttle(
        #                     1, "Critical battery: " + str(battery_percentage) + "%")
        #             else:
        #                 rospy.logfatal_throttle(
        #                     0.1, "Empty battery: " + str(battery_percentage) + "%")
        # else:
        #     rospy.logwarn("Packet lost!")

    def log_changes(self):

         # battery status
        # drone_battery = self.drone.get_state(ReturnHomeBatteryCapacity)
        # self.pub_height.publish(drone_height['altitude'])
        
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


        # r = drone_rpy['roll']
        # p = drone_rpy['pitch']
        # y = drone_rpy['yaw']

        # rospy.loginfo(drone_battery)

    # def yuv_main(self):
    #     with self.flush_queue_lock:
    #         try:
    #             yuv_frame = self.frame_queue.get_nowait()
    #         except queue.Empty:
    #             print("Queue Empty!")

    #         try:
    #             self.yuv_callback(yuv_frame)
    #         except Exception:
    #             # Continue popping frame from the queue even if it fails to show one frame
    #             # traceback.print_exc()
    #             pass
    #         finally:
    #             # Unref the yuv frame to avoid starving the video buffer pool
    #             yuv_frame.unref()
    #             pass

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

    def _altitude(self, msg):
        # rospy.loginfo("target: %f", self.target_altitude)
        kp_alt = 15
        # if self.trigger_height:
        #     if self.emergency_land:
        #         self.drone(PCMD(0, 0, 0, 0, 0, 0))
        #         self.drone(Landing()).wait()
        #     else:
        #         if (time.time() - self.takeoffTime) > 3:

        #             err = self.target_altitude - (msg.data-self.height_offset)
        #             self.P_gaz = float(kp_alt * err)

        #             gaz = self.clamp(self.P_gaz/kp_alt, -1, 1, -30, 30)

        #             self.drone(PCMD(0, 0, 0, 0, gaz, 0))

        #             if (time.time() - self.takeoffTime) > 15 or err < 0.008:
        #                 self.trigger_height = False
        #                 self.manualTime = time.time()
        #                 self.drone(PCMD(0, 0, 0, 0, 0, 0))

        # elif self.manual_control and self.trigger_height is False:
        #     self.target_altitude = msg.data
        #     current_height = None
        #     if time.time() - self.manualTime > 1:
        #         current_height = msg.data
        #         rospy.loginfo("1 second")

        #     if current_height is not None:
        #         self.kp_alt = 6.5
        #         err = self.target_altitude - \
        #             (current_height-self.height_offset)
        #         self.P_gaz = float(kp_alt * err)

        #         rospy.loginfo("PID_altitude controlling")
        #         self.manualTime = time.time()

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
            # app.yuv_main()
            # app.press()
            # rospy.loginfo("Bool-------------------------------------")
            # rospy.loginfo(app.trigger_height)
            rate.sleep()
            # rospy.spin()
            # pass
    except KeyboardInterrupt:
        # app._land(2)
        pass

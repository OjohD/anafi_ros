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
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from std_msgs.msg import Int16, Time, Header, Float32, UInt8
from geometry_msgs.msg import Twist, PointStamped, Vector3Stamped, QuaternionStamped, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

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
        self.pub_odometry = rospy.Publisher(
            "/anafi/odometry", Odometry, queue_size=1)
        self.pub_image = rospy.Publisher("/anafi/image", Image, queue_size=1)
        self.pub_horizontalspeed = rospy.Publisher(
            "/anafi/horizontal_speed", Float32, queue_size=1)

        # Subscribers
        takeoff_sub = rospy.Subscriber("/anafi/takeoff", Int16, self._takeoff)
        land_sub = rospy.Subscriber("/anafi/land", Int16, self._land)
        cmdVel_sub = rospy.Subscriber("/anafi/cmd_vel", Twist, self._cmdVel)

        # subscribe to speed
        speed_sub = rospy.Subscriber(
            "/anafi/speed", Vector3Stamped, self.compute_PID)

        self.bridge = CvBridge()
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        self.user = Int16()
        self.status = False
        self.connected = False

        DRONE_IP = '10.202.0.1'  # SIM
        # DRONE_IP = "192.168.42.1"  # REAL
        self.drone = olympe.Drone(DRONE_IP)
        self.connection = self.drone.connect()
        if getattr(self.connection, 'OK') == True:
            print('connected')
            self.connected = True
            # Setup the callback functions to do some live video processing
            self.drone.set_streaming_callbacks(
                raw_cb=self.yuv_frame_cb,
                flush_raw_cb=self.flush_cb)
            self.drone.start_video_streaming()
        else:
            print('No connection')

        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        time.sleep(3)

        # init PID params
        self.previousTime = time.time()
        self.cumError_pitch = 0
        self.cumError_roll = 0
        self.cumError_z = 0
        self.lastError_pitch = 0
        self.lastError_roll = 0

        self.roll = 0
        self.pitch = 0
        # self.yaw = 0

        self.target_pitch = 0
        self.target_roll = 0

    def clamp(self, x,  in_min,  in_max,  out_min,  out_max):
        if x > in_max:
            x = in_max
    
        if x < in_min:
            x = in_min

        res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        return np.int8(res)

    def yuv_frame_cb(self, yuv_frame):
        # source https://github.com/andriyukr/olympe_bridge
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def flush_cb(self):
        # source https://github.com/andriyukr/olympe_bridge
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def yuv_callback(self, yuv_frame):
        # source https://github.com/andriyukr/olympe_bridge
        # Use OpenCV to convert the yuv frame to RGB
        # the VideoFrame.info() dictionary contains some useful information such as the video resolution
        info = yuv_frame.info()
        rospy.logdebug_throttle(10, "yuv_frame.info = " + str(info))
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]  # convert pdraw YUV flag to OpenCV YUV flag
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        # Publish image
        msg_image = self.bridge.cv2_to_imgmsg(cv2frame, "bgr8")
        self.pub_image.publish(msg_image)

        # yuv_frame.vmeta() returns a dictionary that contains additional metadata from the drone (GPS coordinates, battery percentage, ...)
        metadata = yuv_frame.vmeta()
        rospy.logdebug_throttle(10, "yuv_frame.vmeta = " + str(metadata))

        if metadata[1] != None:
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = '/body'

            # timestamp [millisec]
            frame_timestamp = metadata[1]['frame_timestamp']
            msg_time = Time()
            # secs = int(frame_timestamp//1e6), nsecs = int(frame_timestamp%1e6*1e3)
            msg_time.data = frame_timestamp
            self.pub_time.publish(msg_time)

            drone_quat = metadata[1]['drone_quat']  # attitude
            msg_attitude = QuaternionStamped()
            msg_attitude.header = header
            msg_attitude.quaternion = Quaternion(
                drone_quat['x'], -drone_quat['y'], -drone_quat['z'], drone_quat['w'])
            self.pub_attitude.publish(msg_attitude)

            # GPS location [500.0=not available] (decimal deg)
            location = metadata[1]['location']
            msg_location = PointStamped()
            if location != {}:
                msg_location.header = header
                msg_location.header.frame_id = '/world'
                msg_location.point.x = location['latitude']
                msg_location.point.y = location['longitude']
                msg_location.point.z = location['altitude']
                self.pub_location.publish(msg_location)

            # barometer (m)
            ground_distance = metadata[1]['ground_distance']
            self.pub_height.publish(ground_distance)

            speed = metadata[1]['speed']  # opticalflow speed (m/s)
            msg_speed = Vector3Stamped()
            msg_speed.header = header
            msg_speed.header.frame_id = '/world'
            msg_speed.vector.x = speed['north']
            msg_speed.vector.y = -speed['east']
            msg_speed.vector.z = -speed['down']
            self.pub_speed.publish(msg_speed)

            # horizontal Speed (Magnitude) (m/s)
            drone_speed = math.sqrt(
                (msg_speed.vector.x)**2 + (msg_speed.vector.y)**2)
            self.pub_horizontalspeed.publish(drone_speed)

            # air speed [-1=no data, > 0] (m/s)
            air_speed = metadata[1]['air_speed']
            self.pub_air_speed.publish(air_speed)

            # [0=empty, 100=full]
            battery_percentage = metadata[1]['battery_percentage']
            self.pub_battery.publish(battery_percentage)

            msg_pose = PoseStamped()
            msg_pose.header = header
            msg_pose.pose.position = msg_location.point
            msg_pose.pose.position.z = ground_distance
            msg_pose.pose.orientation = msg_attitude.quaternion
            self.pub_pose.publish(msg_pose)

            Rot = R.from_quat(
                [drone_quat['x'], -drone_quat['y'], -drone_quat['z'], drone_quat['w']])
            drone_rpy = Rot.as_euler('xyz')

            msg_odometry = Odometry()
            msg_odometry.header = header
            msg_odometry.child_frame_id = '/body'
            msg_odometry.pose.pose = msg_pose.pose
            msg_odometry.twist.twist.linear.x = math.cos(
                drone_rpy[2])*msg_speed.vector.x + math.sin(drone_rpy[2])*msg_speed.vector.y
            msg_odometry.twist.twist.linear.y = - \
                math.sin(drone_rpy[2])*msg_speed.vector.x + \
                math.cos(drone_rpy[2])*msg_speed.vector.y
            msg_odometry.twist.twist.linear.z = msg_speed.vector.z
            self.pub_odometry.publish(msg_odometry)

            # log battery percentage
            if battery_percentage >= 30:
                if battery_percentage % 10 == 0:
                    rospy.loginfo_throttle(
                        100, "Battery level: " + str(battery_percentage) + "%")
            else:
                if battery_percentage >= 20:
                    rospy.logwarn_throttle(
                        10, "Low battery: " + str(battery_percentage) + "%")
                else:
                    if battery_percentage >= 10:
                        rospy.logerr_throttle(
                            1, "Critical battery: " + str(battery_percentage) + "%")
                    else:
                        rospy.logfatal_throttle(
                            0.1, "Empty battery: " + str(battery_percentage) + "%")
        else:
            rospy.logwarn("Packet lost!")

    def yuv_main(self):
        with self.flush_queue_lock:
            try:
                yuv_frame = self.frame_queue.get(timeout=0.01)
            except queue.Empty:
                print("Queue Empty!")

            try:
                self.yuv_callback(yuv_frame)
            except Exception:
                # Continue popping frame from the queue even if it fails to show one frame
                # traceback.print_exc()
                pass
            finally:
                # Unref the yuv frame to avoid starving the video buffer pool
                yuv_frame.unref()

    def _takeoff(self, msg):
        if msg.data == 1:
            # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.TakeOff
            self.drone(TakeOff() >> FlyingStateChanged(
                state="hovering", _timeout=10)).wait()

    def _land(self, msg):
        if msg.data == 2:
            # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.Landing
            self.drone(Landing()).wait()
            print("Landed")

    # PID conroller
    # Consider making a thread for this

    def compute_PID(self, drone_speed):

        # https://www.teachmemicro.com/arduino-pid-control-tutorial/

        # PID constants
        kp = 3.5
        ki = 0
        kd = 0

        currentTime = time.time()                                  # get current time
        # compute time elapsed from previous computation
        elapsedTime = currentTime - self.previousTime

        drone_speed = math.sqrt((drone_speed.vector.y) **
                                2 + (drone_speed.vector.x)**2)
        # determine error
        error_pitch = self.target_pitch - drone_speed
        error_roll = self.target_roll - drone_speed

        rospy.loginfo("Error x: %f Error y: %f", error_pitch, error_roll)

        self.cumError_pitch = self.cumError_pitch + error_pitch * \
            elapsedTime                            # compute integral
        self.cumError_roll = self.cumError_roll + error_roll * \
            elapsedTime                            # compute integral

        rateError_pitch = (error_pitch - self.lastError_pitch) / \
            elapsedTime               # compute derivative
        rateError_roll = (error_roll - self.lastError_roll) / \
            elapsedTime               # compute derivative

        self.PID_pitch = abs(kp*error_pitch + ki*self.cumError_pitch + \
            kd*rateError_pitch )        # PID output
        self.PID_roll = abs(kp*error_roll + ki*self.cumError_roll + \
            kd*rateError_roll )        # PID output

        self.lastError_pitch = error_pitch
        # remember current error
        self.lastError_roll = error_roll
        # remember current time
        self.previousTime = currentTime

    def _cmdVel(self, msg):

        # https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.PCMD

        roll = msg.linear.y
        pitch = msg.linear.x
        yaw = msg.angular.z
        gaz = msg.linear.z

        # // If roll or pitch value are non-zero, enabel roll/pitch flag
        flag = not ((abs(roll) < 0.001) and (abs(pitch) < 0.001))

        # our target
        self.target_pitch = abs(pitch)
        self.target_roll = abs(roll)

        if roll > 0:
            roll = self.clamp(self.PID_roll, -1, 1, -10, 10)
        if roll < 0:
            roll = self.clamp(-self.PID_roll, -1, 1, -10, 10)
        else:
            roll = int(roll)

        if pitch > 0:
            pitch = self.clamp(self.PID_pitch, -1, 1, -10, 10)
        if pitch < 0:
            pitch = self.clamp(-self.PID_pitch, -1, 1, -10, 10)
        else:
            pitch = int(pitch)

        yaw = self.clamp(yaw, -3.14, 3.14, -30, 30)
        gaz = self.clamp(gaz, -1, 1, -50, 50)

        self.drone(PCMD(flag, -roll, pitch, -yaw, gaz, 0))


if __name__ == '__main__':
    rospy.init_node("anafi_node")
    rate = rospy.Rate(10)
    app = anafi()
    while not rospy.is_shutdown():
        app.yuv_main()
        rate.sleep()
        # rospy.spin()

#!/usr/bin/env python3
import time
import olympe
import rospy

from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from std_msgs.msg import Int16


class anafi():
    def __init__(self):
        takeoff_sub = rospy.Subscriber("/anafi/takeoff", Int16, self._takeoff)
        land_sub = rospy.Subscriber("/anafi/land", Int16, self._land)

        self.user = Int16()
        self.status = False
        self.connected = False
        # DRONE_IP = '10.202.0.1'  # SIM
        DRONE_IP = "192.168.42.1"  # REAL
        self.drone = olympe.Drone(DRONE_IP)
        self.connection = self.drone.connect()
        if getattr(self.connection, 'OK') == True:
            print('connected')
            self.connected = True
        else:
            print('No connection')

        time.sleep(3)

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


if __name__ == '__main__':
    rospy.init_node("anafi_node")
    app = anafi()
    rospy.spin()

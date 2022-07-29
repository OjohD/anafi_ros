#!/usr/bin/env python3
import rospy
import time
import sys

import pygame

from std_msgs.msg import Int16
from anafi_tutorials.msg import piloting_CMD


class anafi():
    def __init__(self):
        self.status = False
        self.pub_takeoff = rospy.Publisher(
            "/anafi/takeoff", Int16, queue_size=1)
        self.pub_land = rospy.Publisher("/anafi/land", Int16, queue_size=1)
        self.pub_control = rospy.Publisher(
            "/anafi/flight_control", piloting_CMD, queue_size=1)

        self.takeoff_land_msg = Int16()
        self.drone_msg = piloting_CMD()

    def main(self, roll, pitch, yaw, thrust):

        state_roll, state_pitch, state_yaw, state_thrust = 0, 0, 0, 0         # rpyt state select
        

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                # quit safely by landing drone
                # self.drone.piloting_pcmd(0, 0, 0, 0, 0.0)
                time.sleep(1)
                self.takeoff_land_msg.data = 2
                self.pub_land.publish(self.takeoff_land_msg)
                rospy.loginfo('Land')
                pygame.display.quit()
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYUP:
                self.drone_msg.roll = 0
                self.drone_msg.pitch = 0
                self.drone_msg.yaw = 0
                self.drone_msg.thrust = 0

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                   # non-blocking PCMD (roll, pitch, yaw, thrust, piloting_time)

                    self.drone_msg.roll = 0
                    self.drone_msg.pitch = 0
                    self.drone_msg.yaw = 0
                    self.drone_msg.thrust = 0

                    time.sleep(1)
                elif event.key == pygame.K_t:

                    self.takeoff_land_msg.data = 1
                    self.pub_takeoff.publish(self.takeoff_land_msg)
                    rospy.loginfo('Takeoff')
                    self.status = True

                elif event.key == pygame.K_SPACE:

                    self.takeoff_land_msg.data = 2
                    self.pub_land.publish(self.takeoff_land_msg)
                    rospy.loginfo('Land')
                    self.status = False

                elif event.key == pygame.K_w:
                    self.drone_msg.pitch = pitch        # Forward

                elif event.key == pygame.K_s:
                    self.drone_msg.pitch = -pitch        # Backward

                elif event.key == pygame.K_a:
                    self.drone_msg.roll = -roll        # left

                elif event.key == pygame.K_d:
                    self.drone_msg.roll = roll        # right

                elif event.key == pygame.K_j:
                    self.drone_msg.yaw = -yaw  # turn left

                elif event.key == pygame.K_l:
                    self.drone_msg.yaw = yaw        # turn right

                elif event.key == pygame.K_e:
                    self.drone_msg.thrust = thrust        # up

                elif event.key == pygame.K_c:
                    self.drone_msg.thrust = -thrust        # down

                # increase/decrease rpyt values
                elif event.key == pygame.K_1:
                    roll, pitch, yaw, thrust = 1, 0, 0, 0         # -/+ rpyt

                elif event.key == pygame.K_2:
                     roll, pitch, yaw, thrust = 0, 1, 0, 0         # -/+ rpyt

                elif event.key == pygame.K_3:
                    roll, pitch, yaw, thrust = 0, 0, 1, 0         # -/+ rpyt

                elif event.key == pygame.K_4:
                     roll, pitch, yaw, thrust = 0, 0, 0, 1         # -/+ rpyt

            if state_roll:
                rospy.loginfo("State roll")
                if event.key == pygame.K_UP:
                    roll += 1
                    time.sleep(0.5)
                    rospy.loginfo("incrementing roll .......")

                elif event.key == pygame.K_DOWN:
                    roll -= 1
                    time.sleep(0.5)

            if state_pitch:
                if event.key == pygame.K_UP:
                    pitch += 1
                    time.sleep(0.5)

                elif event.key == pygame.K_DOWN:
                    pitch -= 1
                    time.sleep(0.5)

            if state_yaw:
                if event.key == pygame.K_UP:
                    yaw += 1
                    time.sleep(0.5)

                elif event.key == pygame.K_DOWN:
                    yaw -= 1
                    time.sleep(0.5)


            if state_thrust:
                if event.key == pygame.K_UP:
                    thrust += 1
                    time.sleep(0.5)

                elif event.key == pygame.K_DOWN:
                    thrust -= 1
                    time.sleep(0.5)


        self.pub_control.publish(self.drone_msg)



if __name__ == '__main__':
    
    rospy.init_node("publisher_node")
    rospy.loginfo('Publisher initialized')
    rate = rospy.Rate(100)
    

    pygame.init()
    W, H = 200, 200
    screen = pygame.display.set_mode((W, H))

    app = anafi()

    roll, pitch, yaw, thrust = 10, 10, 10, 10         # actual values


    while not rospy.is_shutdown():
        try:
            pygame.display.flip()
            app.main(roll, pitch, yaw, thrust)
            rate.sleep()
        except KeyboardInterrupt:
            quit()

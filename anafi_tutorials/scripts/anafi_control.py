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
        self.roll, self.pitch, self.yaw, self.thrust = 10, 10, 10, 10         # actual values
        # rpyt state select
        self.state_roll, self.state_pitch, self.state_yaw, self.state_thrust = 0, 0, 0, 0

    def main(self):
        rospy.loginfo("states -- r: %s p: %s y: %s t: %s", str(self.state_roll), str(
            self.state_pitch), str(self.state_yaw), str(self.state_thrust))

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
                if self.state_roll:
                    rospy.loginfo("State roll")
                    if event.key == pygame.K_UP:
                        self.roll += 1
                        rospy.loginfo("incrementing roll .......")

                    if event.key == pygame.K_DOWN:
                        self.roll -= 1

                if self.state_pitch:
                    if event.key == pygame.K_UP:
                        self.pitch += 1

                    elif event.key == pygame.K_DOWN:
                        self.pitch -= 1

                if self.state_yaw:
                    if event.key == pygame.K_UP:
                        self.yaw += 1

                    elif event.key == pygame.K_DOWN:
                        self.yaw -= 1

                if self.state_thrust:
                    if event.key == pygame.K_UP:
                        self.thrust += 1

                    elif event.key == pygame.K_DOWN:
                        self.thrust -= 1

                if event.key == pygame.K_ESCAPE:
                   # non-blocking PCMD (roll, pitch, yaw, thrust, piloting_time)

                    self.drone_msg.roll = 0
                    self.drone_msg.pitch = 0
                    self.drone_msg.yaw = 0
                    self.drone_msg.thrust = 0

                    time.sleep(1)
                if event.key == pygame.K_t:

                    self.takeoff_land_msg.data = 1
                    self.pub_takeoff.publish(self.takeoff_land_msg)
                    rospy.loginfo('Takeoff')
                    self.status = True

                if event.key == pygame.K_SPACE:

                    self.takeoff_land_msg.data = 2
                    self.pub_land.publish(self.takeoff_land_msg)
                    rospy.loginfo('Land')
                    self.status = False

                elif event.key == pygame.K_w:
                    self.drone_msg.pitch = self.pitch        # Forward

                elif event.key == pygame.K_s:
                    self.drone_msg.pitch = -self.pitch        # Backward

                elif event.key == pygame.K_a:
                    self.drone_msg.roll = -self.roll        # left

                elif event.key == pygame.K_d:
                    self.drone_msg.roll = self.roll        # right

                elif event.key == pygame.K_j:
                    self.drone_msg.yaw = -self.yaw  # turn left

                elif event.key == pygame.K_l:
                    self.drone_msg.yaw = self.yaw        # turn right

                elif event.key == pygame.K_e:
                    self.drone_msg.thrust = self.thrust        # up

                elif event.key == pygame.K_c:
                    self.drone_msg.thrust = -self.thrust        # down

                # # increase/decrease rpyt values
                if event.key == pygame.K_1:
                    rospy.loginfo("State Key 1 pressed")
                    # -/+ rpyt
                    self.state_roll, self.state_pitch, self.state_yaw, self.state_thrust = 1, 0, 0, 0

                if event.key == pygame.K_2:
                    rospy.loginfo("State Key 2 pressed")
                    # -/+ rpyt
                    self.state_roll, self.state_pitch, self.state_yaw, self.state_thrust = 0, 1, 0, 0

                if event.key == pygame.K_3:
                    rospy.loginfo("State Key 3 pressed")
                    # -/+ rpyt
                    self.state_roll, self.state_pitch, self.state_yaw, self.state_thrust = 0, 0, 1, 0

                if event.key == pygame.K_4:
                    rospy.loginfo("State Key 4 pressed")
                    # -/+ rpyt
                    self.state_roll, self.state_pitch, self.state_yaw, self.state_thrust = 0, 0, 0, 1
            else:
                continue

        self.pub_control.publish(self.drone_msg)


if __name__ == '__main__':

    rospy.init_node("publisher_node")
    rospy.loginfo('Publisher initialized')
    rate = rospy.Rate(100)

    pygame.init()
    W, H = 200, 200
    screen = pygame.display.set_mode((W, H))

    app = anafi()

    while not rospy.is_shutdown():
        try:
            pygame.display.flip()
            app.main()
            rate.sleep()
        except KeyboardInterrupt:
            quit()

#!/usr/bin/env python3
from pickletools import uint8
from tkinter import font
import rospy
import time
import sys

import pygame
from datetime import datetime
import threading


from std_msgs.msg import Int16, Float32, UInt8
from anafi_tutorials.msg import piloting_CMD


class anafi():
    def __init__(self):
        self.status = False
        self.pub_takeoff = rospy.Publisher(
            "/anafi/takeoff", Int16, queue_size=1)
        self.pub_land = rospy.Publisher("/anafi/land", Int16, queue_size=1)
        self.pub_control = rospy.Publisher(
            "/anafi/flight_control", piloting_CMD, queue_size=1)
        self.pub_savedata = rospy.Publisher(
            "/anafi/save_data", Int16, queue_size=1)

        self.sub_droneSpeed = rospy.Subscriber(
            "/anafi/horizontal_speed", Float32, self.get_droneSpeed)

        self.sub_battery = rospy.Subscriber(
            "/anafi/battery", UInt8, self.get_battery_percentage)

        self.stop_threat = False
        self.takeoff_land_msg = Int16()
        self.drone_msg = piloting_CMD()
        self.save_msg = Int16()        
        self.previous_time = time.time()
        self.elapsed = 0
        self.roll, self.pitch, self.yaw, self.thrust = 10, 100, 50, 100         # actual values
        # rpyt state select
        self.state_roll, self.state_pitch, self.state_yaw, self.state_thrust = 0, 0, 0, 0

        # Display data
        self.screen_text_RPYT_States_Select = " "
        self.screen_text_RPYT_Atual_Values = " "
        self.screen_text_drone_speed = " "
        self.battery_percenatge = " "

    def get_droneSpeed(self, msg):
        self.screen_text_drone_speed = str("{:.3f}".format(msg.data)) + " m/s"

    def get_battery_percentage(self, msg):
        self.battery_percenatge = str(msg.data) + " %"

    def main(self):
        self.screen_text_RPYT_States_Select = ("rpyt states: " + str(self.state_roll) + " " + str(
            self.state_pitch) + " " + str(self.state_yaw) + " " + str(self.state_thrust))

        self.screen_text_RPYT_Atual_Values = ("rpyt values: " + str(self.roll) + " " + str(
            self.pitch) + " " + str(self.yaw) + " " + str(self.thrust))

        rospy.loginfo("states -- r: %s p: %s y: %s t: %s", str(self.state_roll), str(
            self.state_pitch), str(self.state_yaw), str(self.state_thrust))

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                # quit safely by landing drone

                self.drone_msg.roll = 0
                self.drone_msg.pitch = 0
                self.drone_msg.yaw = 0
                self.drone_msg.thrust = 0
                self.pub_control.publish(self.drone_msg)
                # time.sleep(1)
                self.stop_threat = True
                self.takeoff_land_msg.data = 2
                self.pub_land.publish(self.takeoff_land_msg)
                rospy.loginfo('Land')
                # pygame.display.quit()
                # pygame.quit()
                # sys.exit()

            elif event.type == pygame.KEYUP:
                # self.drone_msg.roll = 0
                # self.drone_msg.pitch = 0
                self.drone_msg.yaw = 0
                self.drone_msg.thrust = 0
                pass

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
                    self.drone_msg.roll = 0
                    self.drone_msg.pitch = 0
                    self.drone_msg.yaw = 0
                    self.drone_msg.thrust = 0
                    self.pub_control.publish(self.drone_msg)
                    self.takeoff_land_msg.data = 2
                    self.pub_land.publish(self.takeoff_land_msg)
                    rospy.loginfo('Land')
                    self.status = False

                elif event.key == pygame.K_w:
                    self.drone_msg.pitch = self.pitch        # Forward
                    self.save_msg.data = 1
                    self.previous_time = time.time()
                elif event.key == pygame.K_s:
                    self.drone_msg.pitch = -self.pitch        # Backward
                    self.save_msg.data = 1
                    self.previous_time = time.time()
                elif event.key == pygame.K_a:
                    self.drone_msg.roll = -self.roll        # left
                    self.save_msg.data = 1
                    self.previous_time = time.time()
                elif event.key == pygame.K_d:
                    self.drone_msg.roll = self.roll        # right
                    self.save_msg.data = 1
                    self.previous_time = time.time()
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

        # automate control for some duration
        if self.save_msg.data == 1:
            rospy.loginfo("Start recording")
            self.elapsed = time.time() - self.previous_time
            #########################################################################################
            if self.elapsed > 30:
                self.save_msg.data = 0
                self.previous_time = time.time()
                self.drone_msg.roll = 0
                self.drone_msg.pitch = 0
                self.drone_msg.yaw = 0
                self.drone_msg.thrust = 0
                rospy.loginfo("Stop recording")
        self.pub_savedata.publish(self.save_msg)
        self.pub_control.publish(self.drone_msg)


telemetry = "Anafi flight data"
stop_threat = False


def display_text(text, pos):

    # Flight data to display
    global telemetry
    global stop_threat

    # display settings
    pygame.init()
    W, H = 500, 400
    FPS = 60
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption('Anafi Control Panel')
    t_font = pygame.font.SysFont('timesnewroman', 18)
    clock = pygame.time.Clock()
    color = pygame.Color('white')

    while not stop_threat:

        # Updating flight data
        text = telemetry
        # dt = clock.tick(FPS) / 1000
        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         quit()

        screen.fill(pygame.Color('black'))
        # 2D array where each row is a list of words.
        words = [word.split(' ') for word in text.splitlines()]
        # The width of a space.
        space = t_font.size(' ')[0]
        max_width, max_height = screen.get_size()
        x, y = pos
        for line in words:
            for word in line:
                word_surface = t_font.render(word, 0, color)
                word_width, word_height = word_surface.get_size()
                if x + word_width >= max_width:
                    # Reset the x.
                    x = pos[0]
                    # Start on new row.
                    y += word_height
                screen.blit(word_surface, (x, y))
                x += word_width + space
            # Reset the x.
            x = pos[0]
            # Start on new row.
            y += word_height

        pygame.display.update()


if __name__ == '__main__':

    rospy.init_node("publisher_node")
    rospy.loginfo('Publisher initialized')
    rate = rospy.Rate(100)
    app = anafi()

    # Flight data display thread
    threading.Thread(target=display_text, args=(telemetry, (100, 100))).start()

    while not rospy.is_shutdown():
        try:

            if stop_threat:
                pygame.display.quit()
                pygame.quit()
                sys.exit()

            app.main()
            stop_threat = app.stop_threat

            # Updating the display
            now = datetime.now()
            dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
            telemetry = "ANAFI FLIGHT CONTROL PANEL" + "\n" + "        " + dt_string + "\n" + "  " + "\n" \
                + "RPYT States:        " + app.screen_text_RPYT_States_Select + "\n" \
                + "RPYT Values:      " + app.screen_text_RPYT_Atual_Values + "\n" \
                        + "HS:                 " + app.screen_text_drone_speed  \
                + "\n" \
                + "\n" \
                + " Flight Controls Keys" + "\n" \
                + " Roll     ______________ A / D" + "            Battery" + "\n" \
                + " Pitch    ______________ W / S" + "              " + app.battery_percenatge + "\n" \
                + " Yaw      ______________ J / L" + "\n" \
                + " Thrust   ______________ E / C" + "\n" \
                + " Take Off ______________ T" + "\n" \
                                                            + " Land     ______________ SPACE" + "\n" \
                + " Time    :===:  " + str(app.elapsed) + "\n" \


            rate.sleep()

        except KeyboardInterrupt:
            pygame.display.quit()
            pygame.quit()
            sys.exit()

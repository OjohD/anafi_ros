#! /usr/bin/env python

import time
from matplotlib import pyplot as plt
import os
import datetime

class Plot2:

    def __init__(self):
        self.script_dir = os.path.dirname(__file__)
        self.plot_dir = os.path.join(self.script_dir, "Plots/")

        if not os.path.isdir(self.plot_dir):
            os.makedirs(self.plot_dir)

        self.sampletime = 0.1  # 10hz ros
        self.vel_x = 0
        self.vel_y = 0
        self.vel_yaw = 0

        # for one plot
        self.vel = 0

    def update_x(self, velocity):
        self.vel_x = velocity
        # print("Results x", self.vel_x)

        return self.vel_x

    def update_y(self, velocity):
        self.vel_y = velocity
        # print("Results y", self.vel_y)

        return self.vel_y

    def update_yaw(self, velocity):
        self.vel_yaw = velocity
        # print("Results yaw", self.vel_yaw)

        return self.vel_yaw

    def update(self, velocity):
        self.vel = velocity
        # print("Results yaw", self.vel_yaw)

        return self.vel

    def plotter(self, ax_no, title, pid_setpoint):

        # # Set three parameters of PID and limit output
        pid_setpoint = pid_setpoint
        # # Used to set time parameters
        start_time = time.time()

        # # Visualize Output Results
        setpoint, x, y, orient, dt = [], [], [], [], []

        if ax_no > 1:
            f, (ax1, ax2, ax3) = plt.subplots(
                3, 1,  figsize=(10, 40), sharex=True)

            f.suptitle(title, fontsize=20)
            ax1.set_title('X')
            ax1.set_ylim(bottom=-1.0, top=1.0)
            ax2.set_title('Y')
            ax2.set_ylim(bottom=-1.0, top=1.0)
            ax3.set_title('Yaw')
            ax3.set_ylim(bottom=-1.0, top=1.0)
            one_axis = False
        else:
            f, ax = plt.subplots()
            f.suptitle(title, fontsize=20)

            # ax.set_title('Yaw')
            ax.set_ylim(bottom=-1, top=1)

            one_axis = True

        while True:

           # Setting the time variable dt
            current_time = time.time()

            # The variable temp is used as the output in the whole system, and the difference between the variable temp and the ideal value is used as the input in the feedback loop to adjust the change of the variable power.
            # vel = pid(vel)

            # Output Results
            dt += [current_time - start_time]
            setpoint += [pid_setpoint]

            if one_axis:
                vel = self.update(self.vel)
                x += [vel]

                ax.plot(dt, setpoint, 'r', label='target')
                ax.plot(dt, x, 'b', label='PID_x')
                ax.legend(['target', 'PID'])
            else:
                vel_x = self.update_x(self.vel_x)
                x += [vel_x]

                ax1.plot(dt, setpoint, 'r', label='target')
                ax1.plot(dt, x, 'b', label='PID_x')
                ax1.legend(['target', 'PID'])

                vel_y = self.update_y(self.vel_y)
                y += [vel_y]

                ax2.plot(dt, setpoint, 'r', label='target')
                ax2.plot(dt, y, 'b', label='PID_y')
                ax2.legend(['target', 'PID'])

                vel_orient = self.update_yaw(
                    self.vel_yaw)
                orient += [vel_orient]

                ax3.plot(dt, setpoint, 'r', label='target')
                ax3.plot(dt, orient, 'b', label='PID_Or')
                ax3.legend(['target', 'PID'])

            # Used for initial value assignment of variable temp
            if current_time - start_time > 0:
                pid_setpoint = pid_setpoint

            # Visualization of Output Results

            plt.pause(self.sampletime)
            plt.savefig(self.plot_dir + "{}.png".format(datetime.datetime.fromtimestamp((int(time.time()-start_time)-3))))


    

    
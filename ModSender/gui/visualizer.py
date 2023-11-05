"""
Author       : Hanqing Qi
Date         : 2023-10-24 16:30:43
LastEditors  : Hanqing Qi
LastEditTime : 2023-10-24 19:45:46
FilePath     : /ModSender/visualizer.py
Description  : Simple GUI for ModSender
"""
import time
from math import pi
from random import random

import matplotlib.pyplot as plt
import numpy as np


class SensorGUI:
    def __init__(self, enable_gui=True):
        self.enable_gui = enable_gui

        if not enable_gui:
            return

        # Plotting initialization
        plt.ion()

        self.fig, self.ax = plt.subplots()
        # Set background color to black
        self.fig.patch.set_facecolor("black")
        self.ax.set_facecolor("black")

        self.ax.set_xlim(-1.1, 3)
        self.ax.set_ylim(-1.1, 1.5)
        self.ax.set_aspect("equal", "box")
        self.circle = plt.Circle((0, 0), 1, fill=False, color="white", linewidth=2)
        self.ax.add_artist(self.circle)

        # Remove x ticks and y ticks
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        # Initialize the bars
        bar_width = 0.4
        bar_bottom = -1  # Make sure the bars start from the bottom
        self.cur_height_bar = self.ax.bar(
            1.8, 0, bar_width, color="r", bottom=bar_bottom
        )
        self.des_height_bar = self.ax.bar(
            1.4, 0, bar_width, color="g", bottom=bar_bottom
        )

        self.distance_bar = self.ax.bar(
            2.4, 0.3, bar_width, color="b", bottom=bar_bottom
        )

        self.current_yaw = self.ax.arrow(
            0, 0, 0, 0, head_width=0.1, head_length=0.1, fc="r", ec="r", linewidth=3
        )
        self.desired_yaw = self.ax.arrow(
            0, 0, 0, 0, head_width=0.1, head_length=0.1, fc="g", ec="g", linewidth=3
        )

        self.current_yaw_value = self.ax.text(-0.4, 1.1, "", fontsize=12, color="white")
        self.desired_yaw_value = self.ax.text(-0.4, 1.3, "", fontsize=12, color="white")
        self.distance_value = self.ax.text(-0.4, 1.5, "", fontsize=12, color="white")

        self.current_height_value = self.ax.text(
            1.6, -1.1, "", fontsize=12, color="white"
        )
        self.desired_height_value = self.ax.text(
            1.2, -1.1, "", fontsize=12, color="white"
        )
        self.distance_value = self.ax.text(
            2.0, -1.1, "", fontsize=12, color="white"
        )

        height_label = self.ax.text(1.4, -1.2, "Height", fontsize=12, color="white")
        yaw_label = self.ax.text(-0.1, -1.2, "Yaw", fontsize=12, color="white")
        distance_label = self.ax.text(2.2, -1.2, "Distance", fontsize=12, color="white")

    def _angle_to_coordinates(self, radians: float, radius: float = 1.0) -> tuple:
        """
        @description: Convert an angle to coordinates on the circle
        @param       {*} self: -
        @param       {float} radians: The angle in radians
        @param       {float} radius: The radius of the circle (default: 1.0)
        @return      {tuple} (x, y) coordinates
        """
        x = self.circle.center[0] + radius * np.cos(radians)
        y = self.circle.center[1] + radius * np.sin(radians)
        return x, y

    def update_interface(
        self, cur_yaw: float, des_yaw: float, cur_height: float, des_height: float, distance: float
    ) -> None:
        """
        @description: Update the gui interface
        @param       {*} self: -
        @param       {float} cur_yaw: Current value of yaw
        @param       {float} des_yaw: Desired value of yaw from the controller
        @param       {float} cur_height: Current value of height
        @param       {float} des_height: Desired value of height from the controller
        @return      {*} None
        """
        if not self.enable_gui:
            return

        cur_x, cur_y = self._angle_to_coordinates(cur_yaw)
        des_x, des_y = self._angle_to_coordinates(des_yaw)

        # Remove the previous yaws1
        self.current_yaw.remove()
        self.desired_yaw.remove()

        self.current_yaw = self.ax.arrow(
            0,
            0,
            cur_x,
            cur_y,
            head_width=0.1,
            head_length=0.1,
            fc="r",
            ec="r",
            linewidth=3,
        )
        self.desired_yaw = self.ax.arrow(
            0,
            0,
            des_x,
            des_y,
            head_width=0.1,
            head_length=0.1,
            fc="g",
            ec="g",
            linewidth=3,
        )

        self.cur_height_bar[0].set_height(cur_height / 10 if cur_height > 0 else 0)
        self.des_height_bar[0].set_height(des_height / 10 if des_height > 0 else 0)
        self.distance_bar[0].set_height(distance / 300 if distance > 0 else 0)

        self.current_yaw_value.set_text(
            f"Current: {(cur_yaw % (2*np.pi)) / np.pi * 180:.2f}˚"
        )
        self.current_yaw_value.set_color("r")  # Setting the text color to red

        self.desired_yaw_value.set_text(
            f"Desired: {(des_yaw % (2*np.pi)) / np.pi * 180:.2f}˚"
        )
        self.desired_yaw_value.set_color("g")  # Setting the text color to green

        self.desired_height_value.set_text(
            f"D{des_height if des_height > 0 else 0:.2f}"
        )
        self.desired_height_value.set_position((1.2, max(des_height / 10 - 0.9, -0.9)))
        self.desired_height_value.set_color("g")  # Setting the text color to green

        # Display height text
        self.current_height_value.set_text(
            f"C{cur_height if cur_height > 0 else 0:.2f}"
        )
        self.current_height_value.set_position((1.6, max(cur_height / 10 - 0.9, -0.8)))
        self.current_height_value.set_color("r")  # Setting the text color to red

        # Display distance value
        self.distance_value.set_text("{:06d}".format(int(distance)))
            #f"{distance if distance > 0 else 0:10.2f}"        )
        self.distance_value.set_position((2.2,-1.4))
        self.distance_value.set_color("b")  # Setting the text color to red




        plt.draw()


    def sleep(self, delay=0.05):
        """
        Wait using plt
        :param delay: time to wait
        """
        if self.enable_gui:
            plt.pause(delay)
        else:
            time.sleep(delay)


if __name__ == "__main__":
    mygui = SensorGUI(True)

    # Test plotting with increasing numbers
    for i in range(100):
        mygui.update_interface(i*2*pi/100, pi*random()/6, i*0.2, 0)
        mygui.sleep()

    plt.ioff()
    plt.show()

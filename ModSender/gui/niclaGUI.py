"""
Author       : Hanqing Qi
Date         : 2023-10-24 16:30:43
LastEditors  : Hanqing Qi
LastEditTime : 2023-10-24 19:45:46
FilePath     : /ModSender/simpleGUI.py
Description  : Simple GUI for ModSender
"""
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import time


class SimpleGUI:
    def __init__(self):
        # Plotting initialization
        plt.ion()

        self.fig, self.ax = plt.subplots()
        # Set background color to black
        self.fig.patch.set_facecolor("black")
        self.ax.set_facecolor("black")

        self.ax.set_xlim(-1.1, 2.1)
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

        self.current_yaw = self.ax.arrow(
            0, 0, 0, 0, head_width=0.1, head_length=0.1, fc="r", ec="r", linewidth=3
        )
        self.desired_yaw = self.ax.arrow(
            0, 0, 0, 0, head_width=0.1, head_length=0.1, fc="g", ec="g", linewidth=3
        )

        self.current_yaw_value = self.ax.text(-0.4, 1.1, "", fontsize=12, color="white")
        self.desired_yaw_value = self.ax.text(-0.4, 1.3, "", fontsize=12, color="white")

        self.current_height_value = self.ax.text(
            1.6, -1.1, "", fontsize=12, color="white"
        )
        self.desired_height_value = self.ax.text(
            1.2, -1.1, "", fontsize=12, color="white"
        )

        hight_label = self.ax.text(1.4, -1.2, "Height", fontsize=12, color="white")
        yaw_label = self.ax.text(-0.1, -1.2, "Yaw", fontsize=12, color="white")
        # Adding an enclosing square for Nicla Detection visuals
        self.enclosing_square = patches.Rectangle(
            (0, 0),  # will be updated in update_interface
            1,  # width w, will be updated in update_interface
            1,  # height h, will be updated in update_interface
            linewidth=1,
            edgecolor='yellow',
            facecolor='none',
            zorder=4  # ensure it's drawn below the Nicla Detection visuals
        )
        self.ax.add_patch(self.enclosing_square)
        # Adding the Nicla Detection Rectangle
        self.nicla_rect = patches.Rectangle(
            (0, 0),  # (x, y)
            1,  # width w
            1,  # height h
            linewidth=1,
            edgecolor='blue',
            facecolor='none',
            label='Nicla Detection',
            zorder=3  # ensure it's drawn on top
        )
        self.ax.add_patch(self.nicla_rect)
        
        # plt.show()

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
        self,
        cur_yaw: float,
        des_yaw: float,
        cur_height: float,
        des_height: float,
        x: float = 3,
        y: float = 3,
        w: float = 6,
        h: float = 6,
        max_x: float = 240,
        max_y: float = 160
    ) -> None:
        """
        @description: Update the GUI interface, including the Nicla Detection rectangle
        @param       {*} self: -
        @param       {float} cur_yaw: Current value of yaw
        @param       {float} des_yaw: Desired value of yaw from the controller
        @param       {float} cur_height: Current value of height
        @param       {float} des_height: Desired value of height from the controller
        @param       {float} x: x-coordinate of the bottom left corner of the Nicla Detection rectangle
        @param       {float} y: y-coordinate of the bottom left corner of the Nicla Detection rectangle
        @param       {float} w: width of the Nicla Detection rectangle
        @param       {float} h: height of the Nicla Detection rectangle
        @param       {float} max_x: Maximum x-value for the axes
        @param       {float} max_y: Maximum y-value for the axes
        @return      {*} None
        """
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

        self.current_height_value.set_text(
            f"C{cur_height if cur_height > 0 else 0:.2f}"
        )
        self.current_height_value.set_position((1.6, max(cur_height / 10 - 0.9, -0.9)))
        self.current_height_value.set_color("r")  # Setting the text color to red
        # Update the Nicla Detection rectangle and label
        self.nicla_rect.set_xy(((x - w/2)/max_x, (y - h/2)/max_y))
        self.nicla_rect.set_width(w/max_x)
        self.nicla_rect.set_height(h/max_y)
        

        # Optionally, set axes limits
        # self.ax.set_xlim(0, 1)
        # self.ax.set_ylim(0, 1)


        plt.draw()



if __name__ == "__main__":
    mygui = SimpleGUI()
    plt.show()
    try:
        while True:
            mygui.update_interface(0,0,0,0)
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    # while True:
    #     # outputs, y = joyhandler.get_outputs()
    #     # outputs = [0]*13
    #     # feedback = esp_now.getFeedback(1)
    #     # mygui.update_interface(feedback[3], outputs[6], feedback[0], outputs[3])
    #
    #     mygui.update_interface(0,0,0,0)
    #     # esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
    #     # print(feedback)
    #
    #     time.sleep(0.1)
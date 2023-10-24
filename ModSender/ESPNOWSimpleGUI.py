"""
Author       : Hanqing Qi
Date         : 2023-10-21 18:21:56
LastEditors  : Hanqing Qi
LastEditTime : 2023-10-24 08:52:28
FilePath     : /ModSender/ESPNOWSimpleGUI.py
Description  : ESPNOW Library with simple GUI support
"""

# NOTE: In order to use this library, change the improt from ESPNOW to ESPNOWSimpleGUI and add the GUI flag.

import matplotlib.pyplot as plt
import numpy as np
import serial
import time
import re

NULL_ADDRESS = ["00:00:00:00:00:00"]  # Default value for broadcast mode
DELIMITER = "|"  # Delimiter for the message
pattern = r"Flag=(-?\d+), Values=\|([-?\d.,]+)\|"


# ESP-NOW Control Class
class ESPNOWControl:
    def __init__(
        self,
        serial_port: str,
        mac_addresses: list = NULL_ADDRESS,
        ESP_VERBOSE: bool = True,
        ESP_GUI: bool = True,
    ) -> None:
        """
        @description: Constructor for the ESP-NOW Control class
        @param       {*} self: -
        @param       {str} serial_port: The serial port that connects to the sender ESP32
        @param       {list} mac_addresses: The list of MAC addresses of slaves
        @param       {bool} ESP_VERBOSE: If display the feedback data
        @param       {bool} ESP_GUI: If display the GUI
        @return      {*} None
        """
        self.verbose = ESP_VERBOSE
        self.gui = ESP_GUI
        self.feed_flag = 0
        self.feedback = [0, 0, 0, 0, 0, 0]

        if self._init_serial(serial_port):
            print("Serial connection established")
        else:
            raise Exception("Serial connection failed")

        self._send_mac_addresses(mac_addresses)
        print("ESP-NOW Control Initialized Successfully")

        self.broadcast_mode = False
        if mac_addresses == NULL_ADDRESS:
            print("No MAC addresses provided, broadcast mode enabled")
            self.broadcast_mode = True

        if self.gui:
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

            self.current_yaw_value = self.ax.text(
                -0.4, 1.1, "", fontsize=12, color="white"
            )
            self.desired_yaw_value = self.ax.text(
                -0.4, 1.3, "", fontsize=12, color="white"
            )

            self.current_height_value = self.ax.text(
                1.6, -1.1, "", fontsize=12, color="white"
            )
            self.desired_height_value = self.ax.text(
                1.2, -1.1, "", fontsize=12, color="white"
            )

            hight_label = self.ax.text(1.4, -1.2, "Height", fontsize=12, color="white")
            yaw_label = self.ax.text(-0.1, -1.2, "Yaw", fontsize=12, color="white")

    def _init_serial(self, serial_port: str) -> bool:
        """
        @description: Initialize the serial connection
        @param       {*} self: -
        @param       {str} serial_port: The serial port to connect to
        @return      {bool} True if the connection is successful, False otherwise
        """
        try:
            self.serial = serial.Serial(serial_port, 115200, timeout=1)
            print(f"Connected to port {serial_port}")
            while self.serial.in_waiting:  # Clear the buffer
                self.serial.readline().decode(errors="ignore").strip()
            time.sleep(1)
            return True
        except serial.SerialException as e:
            print(f"Failed to connect to port {serial_port}. Error: {e}")
            return False

    def _send_mac_addresses(self, mac_addresses: list) -> None:
        """
        @description: Send the MAC addresses to the sender ESP32
        @param       {*} self: -
        @param       {list} mac_addresses: List of MAC addresses to send
        @return      {*} None
        """

        print("Sending MAC addresses...")
        while True:
            mac_data = "${}#{}$".format(len(mac_addresses), "#".join(mac_addresses))
            self.serial.write(mac_data.encode())
            try:
                incoming = self.serial.readline().decode(errors="ignore").strip()
                if incoming == ("Number of addresses: " + str(len(mac_addresses))):
                    print("MAC addresses sent successfully!")
                    break
            except UnicodeDecodeError:
                print("Received malformed data!")
            time.sleep(0.5)

    def send(
        self, control_params: list, brodcast_channel: int, slaveindex: int
    ) -> None:
        """
        @description: Send the control parameters to the receiver ESP32
        @param       {*} self: -
        @param       {list} control_params: 13 control parameters to send
        @param       {int} brodcast_channel: Channel to broadcast to (will be ignored if slaveindex is not -1)
        @param       {int} slaveindex: Index of the slave to send to (will be ignored only if mac_addresses is empty)
        @return      {*} None
        """
        if (
            len(control_params) != 13
        ):  # Check if the number of control parameters is correct
            raise ValueError(
                "Expected 13 control parameters but got {}".format(len(control_params))
            )
        raw_massage = control_params.copy()
        if (
            self.broadcast_mode or slaveindex == -1
        ):  # Empty mac_addresses or slaveindex is -1
            raw_massage.append(brodcast_channel)
            raw_massage.append(-1)
        else:  # Mac addresses are provided and slaveindex is not -1
            raw_massage.append(-1)
            raw_massage.append(slaveindex)
        # Format the message
        message = str("<" + DELIMITER.join(map(str, raw_massage)) + ">")
        self.serial.write(message.encode())
        try:
            incoming = self.serial.readline().decode(errors="ignore").strip()
            match = re.search(pattern, incoming)
            if match:
                self.feed_flag = int(
                    match.group(1)
                )  # Extract and convert the flag to an integer
                values_str = match.group(2)  # Extract the values as a string

                # Split the values string by commas to get a list of floats
                values = [float(val) for val in values_str.split(",")]
                self.feedback = values

                # Format the values for alignment
                aligned_values_str = ", ".join(
                    ["{:<6.1f}".format(val) for val in values]
                )

                # Replace the original values in the 'incoming' string with the aligned values
                incoming = incoming.replace(values_str, aligned_values_str)

            if self.verbose:
                print("Sending ", incoming)
            if self.gui:
                self.update_interface(
                    values[3], control_params[7], values[2], control_params[4]
                )
                plt.show()
            return incoming[-4:] == "cess"
        except UnicodeDecodeError:
            print("Received malformed data!")
            return False
        return False

    def getFeedback(self) -> tuple:
        """
        @description: Get the feedback data
        @param       {*} self: -
        @return      {tuple} (flag, feedback)
        """
        return self.feed_flag, self.feedback

    def close(self) -> None:
        """
        @description: Close the serial connection
        @param       {*} self: -
        @return      {*} None
        """
        if self.serial.is_open:
            self.serial.close()
            print("Serial connection closed.")

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
        self, cur_yaw: float, des_yaw: float, cur_height: float, des_height: float
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

        plt.draw()

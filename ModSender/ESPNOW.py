import serial
import time

NULL_ADDRESS = ["00:00:00:00:00:00"]  # Default value for broadcast mode
DELIMITER = "|"  # Delimiter for the message


# ESP-NOW Control Class
class ESPNOWControl:
    def __init__(self, serial_port: str, mac_addresses: list = NULL_ADDRESS) -> None:
        """
        @description: Initialize the serial connection and send the MAC addresses
        @param       {*} self: -
        @param       {str} serial_port: The serial port to connect to
        @param       {list} mac_addresses: The list of MAC addresses to send
        @return      {*} None
        """
        if self._init_serial(serial_port):
            print("Serial connection established")
        else:
            raise Exception("Serial connection failed")
        self._send_mac_addresses(mac_addresses)  # Send the MAC addresses
        print("ESP-NOW Control Initialized Successfully")
        self.broadcast_mode = False
        if (
            mac_addresses == NULL_ADDRESS
        ):  # If no MAC addresses are provided, broadcast mode is enabled
            print("No MAC addresses provided, broadcast mode enabled")
            self.broadcast_mode = True

    def _init_serial(self, serial_port: str) -> bool:
        """
        @description: Initialize the serial connection
        @param       {*} self: -
        @param       {str} serial_port: The serial port to connect to
        @return      {bool} True if the connection is successful, False otherwise
        """
        try:
            self.serial = serial.Serial(serial_port, 115200)
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
            print("Sending " + incoming)
        except UnicodeDecodeError:
            print("Received malformed data!")

    def close(self) -> None:
        """
        @description: Close the serial connection
        @param       {*} self: -
        @return      {*} None
        """
        if self.serial.is_open:
            self.serial.close()
            print("Serial connection closed.")

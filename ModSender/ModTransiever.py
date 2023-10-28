from parameters import *
from teleop.joystickHandler import JoystickHandler
from comm.ESPNOW import ESPNOWControl
from robot.robotConfig import RobotConfig
from gui.visualizer import SensorGUI
import time

# User interface
mygui = SensorGUI()
mygui.sleep(0.02)

# Joystick
joyhandler = JoystickHandler()
# Communication
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)

# Load robot configuration
robConfig = RobotConfig(esp_now, ROBOT_CONFIG_FILE)
# Set configs for all slave indexes that you want to use
# Bicopter basic contains configs for a robot with no feedback
robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
# robConfig.sendSetupFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
robConfig.startTranseiver(BRODCAST_CHANNEL, SLAVE_INDEX, MASTER_MAC)  # Start communication

robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)  # Configure IMU
robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)  # Configure Barometer


###### Communicate until Y button (Exit) is pressed #####
y_pressed = False
try:
    while not y_pressed:
        outputs, y_pressed = joyhandler.get_outputs()  # get joystick input
        # outputs = [0]*13
        feedback = esp_now.getFeedback(1)  # get sensor data from robot
        print(feedback)

        mygui.update_interface(feedback[3], outputs[6], feedback[0], outputs[3])  # display sensor data

        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)  # send control command to robot


        # time.sleep(0.02)
        mygui.sleep(0.02)

except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()

from parameters import *
from teleop.joystickHandler import JoystickHandler
from comm.ESPNOW import ESPNOWControl
from robot.robotConfig import RobotConfig
from gui.visualizer import SensorGUI
import time
import numpy as np

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

angle = 0
angle_r = 0

flag = False
y = False
try:
    while not y:

        outputs, y= joyhandler.get_outputs()
        a = joyhandler.a_state

        feedback = esp_now.getFeedback(1)

        if a:
            # print(outputs)
            outputs[1] = 0.2
            if feedback[5] < 400:
                flag = True
                time_elapse = time.time()
                angle_r = np.random.uniform(0, 2 * np.pi)
                joyhandler.tz += angle_r
                while (time.time() - time_elapse) < 3:
                    outputs, y = joyhandler.get_outputs()
                    a = joyhandler.a_state
                    mygui.update_interface(feedback[3], outputs[6], feedback[0], outputs[3])  # display sensor data
                    mygui.sleep(0.02)
                    esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
                    print(feedback)
                    print(outputs)
                    print("turning")


        # outputs[]

        # esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)


        # print(outputs[6])
        # print(outputs[1])

        # # # desired distance
        # dd = 2000
        # #
        # feedback[1] = feedback[1]/100
        # b, ang = feedback[0:2]
        # # ang = ang/100
        #
        # kp_rot = 0.5
        # kp_forward = .0
        # fx = kp_forward * (dd - b)
        # torque = kp_rot * (-0.2 - ang)
        #
        # # print(fx, torque)
        #
        # outputs[1] += fx
        # outputs[6] += torque
        #
        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
        print(feedback)
        print(outputs)

        mygui.update_interface(feedback[3], outputs[6], feedback[0], outputs[3])  # display sensor data
        # line = esp_now.read_from_serial_port()
        # print(line)
        # esp_now.send(outputs, BRODCAST_CHANNEL, 0)
        mygui.sleep(0.02)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()
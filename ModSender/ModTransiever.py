from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
import time
import numpy as np

#ESPNOW PARAMS
ESP_VERBOSE = False
PORT = "COM9"
LIST_OF_MAC_ADDRESS = [
    "34:85:18:91:BC:94", # 0
    "34:85:18:91:BE:34", # 1
    "48:27:E2:E6:EC:CC", #2 Sensor test drone
    "48:27:E2:E6:E4:0C", #3 Big diego drone first
    "48:27:E2:E6:DF:A0", #4 KKL Nicla drone
    "48:27:E2:E6:ED:24", #5 bingxu
    "48:27:E2:E6:DE:3C", #6
    "DC:54:75:D7:F7:FC", #7 hanqing
    "48:27:E2:E6:E6:44", #8 kim
    "34:85:18:91:24:F0", #9
    "34:85:18:8D:86:70", # 10 Sensor Diego
    "34:85:18:8F:36:B0", # 11 Diego
    "DC:54:75:D7:F7:A4"
]

MASTER_MAC = "34:85:18:8D:86:70" # 10 Sensor Diego
SLAVE_INDEX = 11 #-1 means broadcast


BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)
robConfig = RobotConfig(esp_now, "robot_configs.json")

#set configs for all slave indexes that you want to use 
#bicopter basic contains configs for a robot with no feedback
robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
#robConfig.sendSetupFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")

robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)
robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)
# robConfig.startTranseiver(BRODCAST_CHANNEL, SLAVE_INDEX, MASTER_MAC)



######################################################

# MASTER_MAC = "34:85:18:8D:86:70", # 10 Sensor Diego
# SLAVE_INDEX = 10 #-1 means broadcast

# sensor_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)
# robConfig = RobotConfig(sensor_now, "robot_configs.json")
# robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
# robConfig.startTranseiver(BRODCAST_CHANNEL, SLAVE_INDEX, MASTER_MAC)

angle = 0
angle_r = 0

flag = False
y = False
try:
    while not y:

        outputs, y, a = joyhandler.get_outputs()

        flag, feedback = esp_now.getFeedback()

        if a:
            # print(outputs)
            outputs[1] = 0.2
            if feedback[0] < 400:
                flag = True
                time_elapse = time.time()
                angle_r = np.random.uniform(0, 2 * np.pi)
                joyhandler.tz += angle_r
                while (time.time() - time_elapse) < 3:
                    outputs, y, a = joyhandler.get_outputs()
                    time.sleep(0.08)
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
        # line = esp_now.read_from_serial_port()
        # print(line)
        # esp_now.send(outputs, BRODCAST_CHANNEL, 0)
        time.sleep(0.08)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()
from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
import time

#ESPNOW PARAMS
PORT = "COM9"
LIST_OF_MAC_ADDRESS = [
    "34:85:18:91:BC:94", # 0
    "34:85:18:91:BE:34", # 1
    "34:85:18:8D:86:70", # 2 Sensor
    "34:85:18:8F:36:B0", # 3 Diego
    "48:27:E2:E6:E4:0C", # 4 Zhenyu
    "34:85:18:91:20:A8", # 5 Leo
    "34:85:18:AB:ED:C0", # 6 Sender
    "34:85:18:91:24:F0", # 7 runlin
    "DC:54:75:D7:F7:A4",  # 8 Tony
    "48:27:E2:E6:DF:A0"  # 9 Karen
]


SLAVE_INDEX = 3
#-1 means broadcast


BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS)
robConfig = RobotConfig(esp_now, "robot_configs.json")

#set configs for all slave indexes that you want to use 
#bicopter basic contains configs for a robot with no feedback
robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
# robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic_karen")
# robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic_zhenyu")
# robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopter_leo")
#robConfig.sendSetupFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")

robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)
robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)

y = False
try:
    while not y:

        outputs, y = joyhandler.get_outputs()

        # print(outputs)
        
        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
        #esp_now.send(outputs, BRODCAST_CHANNEL, 0)
        time.sleep(0.02)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()
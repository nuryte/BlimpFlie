from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
import time

#ESPNOW PARAMS
PORT = "COM5"
LIST_OF_MAC_ADDRESS = [
    "34:85:18:91:BC:94",
    "34:85:18:91:BE:34",
]
SLAVE_INDEX = 0 #-1 means broadcase
BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if it is not -1


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS)
robConfig = RobotConfig(esp_now, "ModSender\\robot_configs.json")
robConfig.sendAllFlags(1,1,"bicopterbasic")

y = False
try:
    while not y:

        outputs, y = joyhandler.get_outputs()
        
        esp_now.send(outputs, BRODCAST_CHANNEL, 1)
        #esp_now.send(outputs, BRODCAST_CHANNEL, 1)
        time.sleep(0.02)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.close()
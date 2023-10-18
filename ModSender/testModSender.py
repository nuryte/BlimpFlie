from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
import time

#ESPNOW PARAMS
PORT = "COM5"
LIST_OF_MAC_ADDRESS = [
    "34:85:18:91:BC:94",
    "34:85:18:91:BE:34",
    "48:27:E2:E6:EC:CC", #Sensor test drone
    "48:27:E2:E6:E4:0C", #Big diego drone first
    "48:27:E2:E6:DF:A0", #KKL Nicla drone
    "48:27:E2:E6:ED:24", #bingxu
    "48:27:E2:E6:DE:3C",
    
]

MASTER_MAC = "34:85:18:91:C7:80" #address of the transiever

SLAVE_INDEX = 5 #-1 means broadcast


BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS)
robConfig = RobotConfig(esp_now, "ModSender\\robot_configs.json")

#set configs for all slave indexes that you want to use 
#bicopter basic contains configs for a robot with no feedback
robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
#robConfig.sendSetupFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")

#robConfig.startTranseiver(BRODCAST_CHANNEL, SLAVE_INDEX, MASTER_MAC)
# robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)
# robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)

y = False
try:
    while not y:

        outputs, y = joyhandler.get_outputs()
        
        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
        #esp_now.send(outputs, BRODCAST_CHANNEL, 0)
        time.sleep(0.02)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()
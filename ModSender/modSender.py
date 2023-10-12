from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
import time

#ESPNOW PARAMS
PORT = "COM20"
LIST_OF_MAC_ADDRESS = [
    "a3:2f:67:b2:45:89",
    "7e:14:c2:9b:d3:56",
    "b0:92:4c:a1:78:e3",
    "5f:13:87:d0:69:72",
    "4a:22:3b:e9:50:61",
    "34:85:18:91:B5:B4"
]
SLAVE_INDEX = -1
BRODCAST_CHANNEL = 3 # SLAVE_INDEX will override this value if it is not -1


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS)

y = False
try:
    while not y:

        outputs, y = joyhandler.get_outputs()
        esp_now.send(outputs, BRODCAST_CHANNEL, SLAVE_INDEX)
        time.sleep(0.02)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.close()
from parameters import *
from teleop.joystickHandler import JoystickHandler
from comm.ESPNOW import ESPNOWControl
from robot.robotConfig import RobotConfig
from gui.visualizer import SensorGUI
import time

#ESPNOW PARAMS

# MASTER_MAC = "34:85:18:91:C7:80" #address of the transiever


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)
robConfig = RobotConfig(esp_now, "./robot_configs.json")

#set configs for all slave indexes that you want to use 
#bicopter basic contains configs for a robot with no feedback
robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
#robConfig.sendSetupFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")

robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)
robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)
robConfig.startTranseiver(BRODCAST_CHANNEL, SLAVE_INDEX, MASTER_MAC)
mygui = SensorGUI()


y = False
try:
    while not y:


      
        outputs, y = joyhandler.get_outputs()
        #outputs = [0]*13
        feedback  = esp_now.getFeedback(1)
        mygui.update_interface(feedback[3], outputs[6], feedback[0], outputs[3])
        
        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
        #print(feedback)
        
        time.sleep(0.02)
except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()
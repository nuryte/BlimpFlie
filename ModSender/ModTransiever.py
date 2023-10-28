from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
from gui.simpleGUI import SimpleGUI
import time
import math

#ESPNOW PARAMS
ESP_VERBOSE = True
PORT = "COM21" #serial port for the transiever
LIST_OF_MAC_ADDRESS = [
    "34:85:18:91:BC:94",
    "34:85:18:91:BE:34",
    "48:27:E2:E6:EC:CC", #2 Sensor test drone
    "48:27:E2:E6:E4:0C", #3 Big diego drone first
    "48:27:E2:E6:DF:A0", #4 KKL Nicla drone
    "48:27:E2:E6:ED:24", #5 bingxu
    "48:27:E2:E6:DE:3C", #6
    "DC:54:75:D7:F7:FC", #7 hanqing
    "48:27:E2:E6:E6:44", #8 kim
    "34:85:18:91:24:F0", #9
    
    
    
]

# MASTER_MAC = "34:85:18:91:C7:80" #address of the transiever
MASTER_MAC = "34:85:18:91:BC:94"

SLAVE_INDEX = 2 #-1 means broadcast


BRODCAST_CHANNEL = 1 # SLAVE_INDEX will override this value if SLAVE_INDEX is not -1


joyhandler = JoystickHandler()
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)
robConfig = RobotConfig(esp_now, "ModSender\\robot_configs.json")

#set configs for all slave indexes that you want to use 
#bicopter basic contains configs for a robot with no feedback
robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
#robConfig.sendSetupFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")

robConfig.startTranseiver(BRODCAST_CHANNEL, SLAVE_INDEX, MASTER_MAC)
robConfig.startBNO(BRODCAST_CHANNEL, SLAVE_INDEX)
robConfig.startBaro(BRODCAST_CHANNEL, SLAVE_INDEX)

mygui = SimpleGUI()


# Nicla vision control
des_yaw = 0
des_height = 0
control_yaw = 0
control_height = 0
Ky = 2

Kh = 1

gamma1 = 0.8
gamma2 = 0.8
max_x = 240
max_y = 160
_yaw = 0
x, y, w, h = -1, -1, -1, -1
detected = False

time_prev = time.time()

toggle_y = False
try:
    while not toggle_y:


      
        outputs, toggle_y = joyhandler.get_outputs(_yaw)
        feedback2 = esp_now.getFeedback(2) 
        print(feedback2[0:4])
        feedback = esp_now.getFeedback(1) 
        # print(feedback)
        _height = feedback[0]
        _yaw = feedback[1]
        _x = feedback[2] * 1000
        _y = feedback[3] * 1000
        _w = feedback[4] * 1000
        _h = feedback[5] * 1000
        
        dt = time.time() - time_prev
        #If the Nicla updates the desired yaw and height
        if x == 0 and y== 0 and w == 0 and h == 0:
            detected = False
        else:
            detected = True
        if x != _x or y != _y or w != _w or h != _h:
            
            x, y, w, h = _x, _y, _w, _h
            des_yaw = des_yaw*gamma1 - (((x - max_x/2)/max_x)*math.pi/2)*(1-gamma1)
            control_yaw = _yaw+ des_yaw * Ky#* .2 + control_yaw* .8
            des_yaw -= Ky * des_yaw * dt
            des_height = des_height*gamma2 + ((y - max_y/2)/max_y)*(1-gamma2)
            control_height = _height
            

        
        # control_yaw += Ky * des_yaw * dt
        # des_yaw -= Ky * des_yaw * dt
        # control_height += Kh * des_height * dt
        # des_height -= Kh * des_height * dt
        outputs[6] = control_yaw
        mygui.update_interface(_yaw, outputs[6],_height,outputs[3])#des_yaw+control_yaw, detected, 0)#control_height, des_height)
        # print("Yaw", _height, "control_yaw", round(control_yaw*180/3.14,2), "des_yaw", round(des_yaw*180/3.14,2))

        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
        #print(feedback)
        time_prev = time.time()
        
        time.sleep(0.02)
except Exception as e:
    print(e)
    if e == KeyboardInterrupt:
        print("Loop terminated by user.")
    else:
        print("Loop terminated by error.")

esp_now.send([0] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()
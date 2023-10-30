from joystickHandler import JoystickHandler
from ESPNOW import ESPNOWControl
from robotConfig import RobotConfig
from gui.niclaGUI import SimpleGUI
import time
import math
def clamp(value, min_val=0.0, max_val=1.0):
    return min(max(value, min_val), max_val)

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
cont = robConfig.sendAllFlags(BRODCAST_CHANNEL, SLAVE_INDEX, "bicopterbasic")
if not cont:
    print("plug in drone!")
    quit()
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
Ky = 1

Kh = 1

gamma1 = 0.5
gamma2 = 0.5
max_x = 240
x_min_cal = 0
x_max_cal = 1
x_cal = 0.5
max_y = 160
_yaw = 0
_height = 0
x, y, w, h = -1, -1, -1, -1
detected = False

time_prev = time.time()

toggle_y = False
toggle_a = False
try:
    while not toggle_y:


      
        outputs, toggle_y, toggle_a = joyhandler.get_outputs(_yaw, _height + 3)
        
        outputs[8] = int(toggle_a)
        feedback2 = esp_now.getFeedback(2) 
        #print(feedback2[0:4])
        feedback = esp_now.getFeedback(1) 
        # print(feedback)
        _height = feedback[0]
        _yaw = feedback[1]
        _x = feedback[2] * 1000
        _y = feedback[3] * 1000
        _w = feedback[4] * 1000
        _h = feedback[5] * 1000
        
        dt = time.time() - time_prev
        x_cal = (x/max_x - x_min_cal)/(x_max_cal - x_min_cal)#,0,1)
        #If the Nicla updates the desired yaw and height
        if _x == 0 and _y== 0 and _w == 0 and _h == 0:
            detected = False
            #right side .7 is max
            #left side .1 is max
        elif x != _x or y != _y or w != _w or h != _h:
            detected = True
            
            des_yaw = ((x_cal - .5)*math.pi/2)
            control_yaw = _yaw- des_yaw * Ky#* .2 + control_yaw* .8
            #des_yaw -= Ky * des_yaw 
            des_height = des_height*gamma2 + ((y - max_y/2)/max_y)*(1-gamma2)
            control_height = _height
            if abs(des_yaw ) < .2:
                outputs[1] = .2
            else:
                outputs[1] = 0

                
        x, y, w, h = _x, _y, _w, _h
            

        
        # control_yaw += Ky * des_yaw * dt
        # des_yaw -= Ky * des_yaw * dt
        # control_height += Kh * des_height * dt
        # des_height -= Kh * des_height * dt
        #outputs[6] = control_yaw
        mygui.update_interface(_yaw, outputs[6],toggle_a,outputs[3], x_cal*max_x, _y, _w, _h)#des_yaw+control_yaw, detected, 0)#control_height, des_height)
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

esp_now.send([21,0] + outputs[:-2], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()
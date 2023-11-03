from autonomy.RandomWalk import RandomWalk
from parameters import *
from teleop.joystickHandler import JoystickHandler
from comm.ESPNOW import ESPNOWControl
from robot.robotConfig import RobotConfig
from gui.visualizer import SensorGUI
import time
import math
def clamp(value, min_val=0.0, max_val=1.0):
    return min(max(value, min_val), max_val)


# Communication
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)

# Load robot configuration
robConfig = RobotConfig(esp_now, ROBOT_CONFIG_FILE, BRODCAST_CHANNEL, SLAVE_INDEX, ROBOT_JASON, MASTER_MAC)
robConfig.InicializationSystem()

# User interface
mygui = SensorGUI(GUI_ENABLED, esp_now, robConfig)
mygui.sleep(0.02)

YAW_SENSOR, Z_SENSOR = robConfig.getFeedbackParams(ROBOT_JASON)
# Joystick
joyhandler = JoystickHandler(yaw_sensor=YAW_SENSOR)


# Autonomous Behavior
autonomous = RandomWalk()
autonomous.begin()

###### Communicate until Y button (Exit) is pressed #####
y_pressed = False

try:
    while not y_pressed:
        outputs, y_pressed = joyhandler.get_outputs()  # get joystick input
        # outputs = [0]*13
        feedback = esp_now.getFeedback(1)  # get sensor data from robot
        # print(feedback)
        '''
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
            

                
        x, y, w, h = _x, _y, _w, _h
        '''
        # ------- Autonomous mode ----------
        a_key_pressed = joyhandler.a_state
        if a_key_pressed:
            des_fx, des_z, des_yaw = autonomous.execute(feedback)
            outputs[1] = des_fx  # Forward
            outputs[3] = des_z  # Z
            joyhandler.tz = des_yaw  # Yaw control

        # Display sensors and output
        mygui.update_interface(feedback[3], outputs[6], feedback[0], outputs[3], feedback[5])  # display sensor data
        # Communicate with robot
        esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, SLAVE_INDEX)  # send control command to robot


        # time.sleep(0.02)
        mygui.sleep(0.02)

except KeyboardInterrupt:
    print("Loop terminated by user.")
esp_now.send([21, 0,0,0,0,0,0,0,0,0,0,0,0], BRODCAST_CHANNEL, SLAVE_INDEX)
esp_now.close()

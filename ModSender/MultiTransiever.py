from autonomy.RandomWalk import RandomWalk
from parameters import *
from teleop.joystickHandler import JoystickHandler
from comm.ESPNOW import ESPNOWControl
from robot.robotConfig import RobotConfig
from gui.visualizer import SensorGUI


# Override feedback params for multiple robots
YAW_SENSOR = True
Z_SENSOR = True


# User interface




# Joystick
joyhandler = JoystickHandler(yaw_sensor=YAW_SENSOR)

# Communication
esp_now = ESPNOWControl(PORT, LIST_OF_MAC_ADDRESS, ESP_VERBOSE)

# For each robot

# Load robot configuration
robotConfigs = [RobotConfig(esp_now, i, robot_mac) for i, robot_mac in enumerate(LIST_OF_MAC_ADDRESS)]
# GUIS
sensor_guis = [SensorGUI(GUI_ENABLED,robConfig=robConfig) for robConfig in robotConfigs]

# Send flags to each robot
for robConfig in robotConfigs:
    # Set configs for all slave indexes that you want to use
    # Bicopter basic contains configs for a robot with no feedback
    print("Connecing to robot %d: "%robConfig.slave_index, robConfig.mac)
    active = robConfig.sendAllFlags(BRODCAST_CHANNEL, ROBOT_JASON)
    if not active:
        quit()

    robConfig.sendAllFlags(BRODCAST_CHANNEL,  ROBOT_JASON)  # Redundant sent.


    if YAW_SENSOR:
        robConfig.startBNO(BRODCAST_CHANNEL)  # Configure IMU

    if Z_SENSOR:
        robConfig.startBaro(BRODCAST_CHANNEL)  # Configure Barometer


    robConfig.startThrustRange(BRODCAST_CHANNEL, "bicopterbasic")  # Motor specifications
    robConfig.startTranseiver(BRODCAST_CHANNEL, MASTER_MAC)  # Start communication


# Autonomous Behavior
behavior_robots = [RandomWalk() for _ in robotConfigs]
for robot_behavior in behavior_robots:
    robot_behavior.begin()

###### Communicate until Y button (Exit) is pressed #####
y_pressed = False
try:
    while not y_pressed:
        outputs, y_pressed, a_key_pressed = joyhandler.get_outputs()  # get joystick input
        # outputs = [0]*13

        for i, robotConfig in enumerate(robotConfigs):
            feedback = esp_now.getFeedback(i)  # get sensor data from robot

             # ------- Autonomous mode ----------
            if a_key_pressed:
                des_fx, des_z, des_yaw = behavior_robots[i].execute(feedback)
                outputs[1] = des_fx  # Forward
                outputs[3] = des_z  # Z
                joyhandler.tz = des_yaw  # Yaw control


            #
            #
            # if (i==1):
            #     print(feedback)

            # Display sensors and output
            sensor_guis[i].update_interface(feedback[1], outputs[6], feedback[0], outputs[3], feedback[2])  # display sensor data
            print(feedback)
            # Send message to all robots
            esp_now.send([21] + outputs[:-1], BRODCAST_CHANNEL, robotConfig.slave_index)  # send control command to robot


        # time.sleep(0.02)
        sensor_guis[0].sleep(0.02)

except KeyboardInterrupt:
    print("Loop terminated by user.")

for robotConfig in robotConfigs:
    esp_now.send([21, 0,0,0,0,0,0,0,0,0,0,0,0], BRODCAST_CHANNEL, robotConfig.slave_index)
esp_now.close()

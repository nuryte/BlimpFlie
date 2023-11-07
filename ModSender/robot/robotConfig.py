import time
import json

from parameters import BRODCAST_CHANNEL, MASTER_MAC, ROBOT_JASON


class RobotConfig:
    def __init__(self, esp_now, slave_index, mac_address):
        self.mac = mac_address
        self.slave_index = slave_index

        self.config_file = "ModSender/config/" + mac_address.replace(":", "")[-4:] + ".json"

        self.esp_now = esp_now


    def get_config(self, CONFIG_INDEX):
        return self.configs.get(str(CONFIG_INDEX), {'feedbackPD': {}, 'weights': {}, 'initflags': {}, 'hardware': {}, 'nicla': {}})
    
    def _fill_with_zeros(self, data, size=13):
        """Fill the given list with zeros until it reaches the specified size."""
        while len(data) < size:
            data.append(0)
        return data

    def _send_data(self, esp_now_input, BRODCAST_CHANNEL):
        """A helper function to encapsulate the repeated sending behavior."""
        esp_now_input = self._fill_with_zeros(esp_now_input)
        count = 0
        while not self.esp_now.send(esp_now_input, BRODCAST_CHANNEL, self.slave_index):
            count += 1
            if count > 20:
                print("Gave up sending on ", self.slave_index)
                return False
            time.sleep(0.05)
        return True

    def sendAllFlags(self, BRODCAST_CHANNEL, CONFIG_INDEX):
        print("send all flags!")

        # Fetch the feedbackPD and weights for the given CONFIG_INDEX
        config = self.get_config(CONFIG_INDEX)
        feedbackPD = config['feedbackPD']
        weights = config['weights']
        hardware = config['hardware']
        nicla = config['nicla']
        

        data_sets = [
            [10, 0, 
             feedbackPD["roll"], 
             feedbackPD["pitch"], 
             feedbackPD["yaw"],
             feedbackPD["x"], 
             feedbackPD["y"], 
             feedbackPD["z"], 
             feedbackPD["rotation"]],
            [11, 0, 
             feedbackPD["Croll"], 
             feedbackPD["Cpitch"], 
             feedbackPD["Cyaw"],
             feedbackPD["Cx"], 
             feedbackPD["Cy"], 
             feedbackPD["Cz"], 
             feedbackPD["Cabsz"]],
            [12, 0, 
             feedbackPD["kproll"], 
             feedbackPD["kdroll"], 
             feedbackPD["kppitch"],
             feedbackPD["kdpitch"], 
             feedbackPD["kpyaw"], 
             feedbackPD["kdyaw"],
             feedbackPD["kiyaw"], 
             feedbackPD["kiyawrate"], 
             feedbackPD["yawRateIntegralRange"],
             feedbackPD["errorYawrateRange"],
             feedbackPD["errorYawRange"]],
            [13, 0, 
             feedbackPD["kpx"], 
             feedbackPD["kdx"], 
             feedbackPD["kpy"],
             feedbackPD["kdy"], 
             feedbackPD["kpz"], 
             feedbackPD["kdz"],
             feedbackPD["lx"], 
             feedbackPD["pitchSign"], 
             feedbackPD["pitchOffset"],
             feedbackPD["rollSign"], 
             feedbackPD["rollOffset"]],
            [14, 0, 
             weights["eulerGamma"], 
             weights["rollRateGamma"], 
             weights["pitchRateGamma"],
             weights["yawRateGamma"], 
             weights["zGamma"], 
             weights["vzGamma"],
             hardware["yawScaleEnable"]],
            [15, 0, 
             feedbackPD["kiz"], 
             feedbackPD["integral_dt"], 
             feedbackPD["z_int_low"],
             feedbackPD["z_int_high"], 
             feedbackPD["servo1offset"], 
             feedbackPD["servo2offset"],
             feedbackPD["rollPitchSwitch"],
             hardware["kf1"],
             hardware["kf2"],
             hardware["maxRadsYaw"],
             hardware["fxyawScale"]],
             [95, 0, 
             nicla["goal_theta_back"], 
             nicla["goal_theta_front"], 
             nicla["goal_dist_thresh"],
             nicla["max_move_x"],
             nicla["goal_ratio"],
             nicla["yaw_move_threshold"]]
        ]

        for data in data_sets:
            time.sleep(.1)
            if not self._send_data(data, BRODCAST_CHANNEL):
                return False

        print("All Flags Sent on ", self.slave_index, " for ", CONFIG_INDEX)
        return True
    

    def sendSetupFlags(self, BRODCAST_CHANNEL, CONFIG_INDEX):
        print("send all flags!")

        # Fetch the feedbackPD and weights for the given CONFIG_INDEX
        config = self.get_config(CONFIG_INDEX)
        
        initflags = config['initflags']
        
        data_sets = [[16, 0, 
             initflags["verbose"], 
             initflags["sensors"], 
             initflags["escarm"],
             initflags["UDP"], 
             initflags["Ibus"], 
             initflags["ESPNOW"],
             initflags["PORT"], 
             initflags["motor_type"], 
             initflags["mode"],
             initflags["control"],
             ]]
        
        for data in data_sets:
            time.sleep(.1)
            if not self._send_data(data, BRODCAST_CHANNEL):
                return False

        print("All inits Sent on ", self.slave_index, " for ", CONFIG_INDEX)
        return True
    
    def startBNO(self, BRODCAST_CHANNEL):
        time.sleep(0.1)
        if not self._send_data([97], BRODCAST_CHANNEL):
            time.sleep(1)
            return False
        time.sleep(1)
    

    def startBaro(self, BRODCAST_CHANNEL):
        time.sleep(0.1)
        if not self._send_data([98], BRODCAST_CHANNEL):
            time.sleep(1)
            return False
        time.sleep(1)
    

    def startTranseiver(self, BRODCAST_CHANNEL, MASTER_MAC):
        time.sleep(0.1)
        # Split the MAC address into its bytes and convert to integers
        mac_bytes = [int(byte, 16) for byte in MASTER_MAC.split(':')]

        # Convert bytes to floats and print
        mac_floats = [float(byte) for byte in mac_bytes]
        print(mac_floats)
        if not self._send_data([17,0] + mac_floats +[self.slave_index], BRODCAST_CHANNEL):
            time.sleep(1)
            return False
        time.sleep(1)

    def startThrustRange(self,BRODCAST_CHANNEL, CONFIG_INDEX):
        time.sleep(0.1)
        # Fetch the feedbackPD and weights for the given CONFIG_INDEX
        config = self.get_config(CONFIG_INDEX)

        initflags = config['initflags']


        if not self._send_data([96, 0, initflags["min_thrust"],
             initflags["max_thrust"]], BRODCAST_CHANNEL):

            time.sleep(1)
            return False
        time.sleep(1)


    def getFeedbackParams(self,CONFIG_INDEX):
        # Fetch the feedbackPD and weights for the given CONFIG_INDEX
        config = self.get_config(CONFIG_INDEX)
        feedbackPD = config['feedbackPD']

        yaw_enabled = feedbackPD["yaw"]
        z_endabled = feedbackPD["z"]

        return yaw_enabled,z_endabled

    def initialize_system(self):
        # Set configs for all slave indexes that you want to use

        with open(self.config_file, 'r') as f:
            self.configs = json.load(f)

        # Bicopter basic contains configs for a robot with no feedback
        active = self.sendAllFlags(BRODCAST_CHANNEL, ROBOT_JASON)
        if not active:
            quit()

        self.sendAllFlags(BRODCAST_CHANNEL, ROBOT_JASON)  # Redundant sent.

        # Configure sensors
        self.startBNO(BRODCAST_CHANNEL)  # Configure IMU
        self.startBaro(BRODCAST_CHANNEL)  # Configure Barometer


        self.startThrustRange(BRODCAST_CHANNEL, "bicopterbasic")  # Motor specifications
        self.startTranseiver(BRODCAST_CHANNEL, MASTER_MAC)  # Start communication


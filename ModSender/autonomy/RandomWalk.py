import time

import numpy as np

from ModSender.autonomy.Autonomous import Autonomous


class RandomWalk(Autonomous):

    def __init__(self):
        self.x_force = 0.3
        self.des_yaw = 0
        self.des_z = 4

        self.counter = 100
        self.flag = False
    def begin(self):
        pass


    def execute(self, feedback):

          # constant x force to move forward

        distance = feedback[5]  # Distance from the sonar

        if distance < 400:  # if distance is less than 400 cm then turn
            time_elapse = time.time()  # save the current time
            # angle_r = np.random.uniform(0, np.pi) + np.pi/2 # random value in the opposite direction
            # joyhandler.tz += angle_r # modify the angle to send
            # self.des_yaw += np.pi

            self.x_force = -0.3
            self.counter = 0
            self.flag = True

        if self.flag and self.counter > 25:
            self.des_yaw += np.random.uniform(0, np.pi) + np.pi/2
            self.x_force = 0.3
            self.counter = 0
            self.flag = False

        print(self.counter)
        self.counter+=1

        des_fx, des_z, des_yaw =  self.x_force, self.des_z, self.des_yaw

        return des_fx, des_z, des_yaw
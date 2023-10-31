import time

import numpy as np

from ModSender.autonomy.Autonomous import Autonomous




class RandomWalk(Autonomous):

    def __init__(self, forward_force=0.3, min_distance=400, des_z=4):
        # Constants
        self.forward_force = forward_force
        self.min_distance = min_distance
        self.des_z = des_z
        self.time_backward = 2
        self.time_rotate = 2

        # Variable
        self.des_yaw = 0

        # current action
        self.current_action = 0

        # actions
        self.actions = [self._action_move_forward, self._action_move_backward, self._action_rotate]

    def begin(self):
        self._restart_timer()



    def execute(self, feedback):

        # Variables to make decisions
        distance = feedback[5]  # Distance from the sonar
        time_elapsed = self._time_elapsed()

        # ---------- Switch actions based on timer and distance -----------
        if self.current_action == 0 and distance < self.min_distance:
            self.current_action = 1  # Move backwards
            self._restart_timer()

        elif self.current_action == 1 and time_elapsed > self.time_backward:
            self.current_action = 2  # Rotate
            self._restart_timer()

        elif self.current_action == 2 and time_elapsed > self.time_rotate:
            self.current_action = 0  # Move forward

        # Select the current action function
        action = self.actions[self.current_action]

        return action()

    def _restart_timer(self):
        self.time_elapse = time.time()

    def _time_elapsed(self):
        return time.time() - self.time_elapse

    def _action_move_forward(self):
        return self.forward_force, self.des_yaw, self.des_z

    def _action_move_backward(self):
        return -self.forward_force, self.des_yaw, self.des_z

    def _action_rotate(self):
        self.des_yaw += np.random.uniform(0, np.pi) + np.pi / 2
        return self.forward_force, self.des_yaw, self.des_z

import time

import numpy as np

from ModSender.autonomy.Autonomous import Autonomous




class DeterministicWalk(Autonomous):

    def __init__(self, forward_force=0.3, min_distance=500, des_z=7):
        # Constants
        self.forward_force = forward_force
        self.min_distance = min_distance
        self.des_z = des_z
        self.time_backward = 2
        self.time_rotate = 2

        # Variable
        self.des_yaw = np.radians(205+90)

        self.step = np.radians(10)

        # current action
        self.current_action = 0

        # actions
        self.actions = [self._action_move_forward, self._action_move_backward, self._action_rotate, self._action_wait]

    def begin(self):
        self._restart_timer()


    def _choose_action(self, feedback):
        # Variables to make decisions
        distance = feedback[2]  # Distance from the sonar
        time_elapsed = self._time_elapsed()

        switch_time = 8
        # ---------- Switch actions based on timer and distance -----------
        if self.current_action == 0 and (distance < self.min_distance or time_elapsed > switch_time):
            self.current_action = 1  # Move backwards
            self._restart_timer()

        elif self.current_action == 1 and time_elapsed > self.time_backward:
            self.current_action = 2  # Rotate
            self._restart_timer()
        elif self.current_action == 2:
            self.current_action = 3  # Wait
        elif self.current_action == 3 and time_elapsed > self.time_rotate:
            self.current_action = 0  # Move forward
            self._restart_timer()

        print("Current action: ", self.current_action)
        return self.actions[self.current_action]

    def execute(self, feedback):

        # Select the current action function
        action = self._choose_action(feedback)

        return action()

    def _restart_timer(self):
        self.time_elapse = time.time()

    def _time_elapsed(self):
        return time.time() - self.time_elapse

    def _action_move_forward(self):
        return self.forward_force, self.des_z, self.des_yaw

    def _action_move_backward(self):
        return -self.forward_force, self.des_z, self.des_yaw

    def _action_rotate(self):
        # self.des_yaw += np.random.uniform(0, np.pi) + np.pi / 2
        self.des_yaw += np.pi
        return 0, self.des_z, self.des_yaw

    def _action_wait(self):
        return 0, self.des_z, self.des_yaw

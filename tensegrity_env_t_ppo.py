import math
import numpy as np

from gym import spaces

import tensegrity_env_torque as tet
import common as cm


class TensegrityEnvTorquePpo2(tet.TensegrityEnvTorque):
    def __init__(self, state):
        super(TensegrityEnvTorquePpo2, self).__init__(state=state, action_space=spaces.Box(np.array([-1]), np.array([1])))
    
    def decodeAction(self, action):
        return action
    
    def compute_reward(self, action, done=None):
        return math.cos(abs(cm.rad2Norm(self.pole_angle_real))) - abs(self.decodeAction(action)) * 0.0001


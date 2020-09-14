import math
import numpy as np

from gym import spaces

import tensegrity_env_delta as ted
import common as cm


class TensegrityEnvDeltaPpo2(ted.TensegrityEnvDelta):
    def __init__(self, state, render=False):
        super(TensegrityEnvDeltaPpo2, self).__init__(state=state, action_space=spaces.Box(np.array([-1]*24), np.array([1]*24)), render=render)
    
    def decodeAction(self, action):
        #for i in range(len(action)):
        #    action[i] = action[i]/10
        return action
    
    
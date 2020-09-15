import math
import numpy as np

from gym import spaces

import tensegrity_env_sine as tes
import common as cm

NAME_POLICY = "ppo2_s_3x_aws_a.zip"

class TensegrityEnvSinePpo2(tes.TensegrityEnvSine):
    def __init__(self, state, render=False):
        super(TensegrityEnvSinePpo2, self).__init__(state=state, 
            action_space=spaces.Box(np.array([-1]*4), np.array([1]*4)),
            observation_space=spaces.Box(np.array([-2] * 9), np.array([2] * 9)),
            render=render)
    
    def decodeAction(self, action):
        #for i in range(len(action)):
        #    action[i] = action[i]/10
        return action
    
    
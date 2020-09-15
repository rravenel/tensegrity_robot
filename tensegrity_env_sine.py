import math
import numpy as np

import pybullet as p

import tensegrity_env_base as teb
import model_sine as m
import common as cm


class TensegrityEnvSine(teb.TensegrityEnvBase):
    def __init__(self, state, action_space, observation_space, render=False):
        super(TensegrityEnvSine, self).__init__(state=state, action_space=action_space, observation_space=observation_space, render=render)
                                    
    def updateSimulation(self, action): 
        m.act(action)
        
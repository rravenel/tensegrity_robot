import math
import numpy as np

import pybullet as p

import tensegrity_env_base as teb
import model as m
import common as cm


class TensegrityEnvDelta(teb.TensegrityEnvBase):
    def __init__(self, state, action_space, render=False):
        super(TensegrityEnvDelta, self).__init__(state=state, action_space=action_space, render=render)
                                    
    def updateSimulation(self, action): 
        m.act(action)
        
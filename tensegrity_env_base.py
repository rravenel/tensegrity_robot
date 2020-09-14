import os
import math
import numpy as np
import time

import gym
from gym import spaces
import pybullet as p
import pybullet_data

import tensegrity as t
import model as m
import common as cm

'''
This version relies on the furuta-v3 conda environment.

Base TensegrityEnv

Implementations MUST define:
    self.action_space
        Defined type is dependent on requirements of implemented RL algorithm

    assign_throttle(self, action)
        Defined type is dependent on requirements of implemented RL algorithm

Implementations MAY define:
    compute_reward(self)
        Overriding this enables differentiation unique to implemented RL algorithm, as well as 
        different strategies, such as whether to separate the spin-up task from balancing

'''
PI = cm.PI # convenience

MODEL_PATH = "model/"

# control by setting motor current
class TensegrityEnvBase(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    def __init__(self, render=False, state=None, action_space=None):
        super(TensegrityEnvBase, self).__init__()
        
        if None is state:
            print("state is None. Exiting.")
            exit()   
        self.state = state
              
        if None is action_space:
            print("action_space is None. Exiting.")
            exit()   
        self.action_space = action_space
        
        obs_min, obs_max = self.setObs()
        self.observation_space = spaces.Box(np.array(obs_min), np.array(obs_max))
                
        #self.setRender(render)

    def setObs(self):
        obs_min = [-2] * 35
        obs_max = [2] * 35
                
        return obs_min, obs_max
    
    # must be called immediately following __init__()
    def setRender(self, render):
        if (render):
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
    
    def reset(self, position=PI, velocity=0, arm_vel=0):
        self.throttle = 0
        self.action = None
        self._envStepCounter = 0

        p.resetSimulation()
        p.setGravity(0,0,-9.8) # m/s^2
        p.setTimeStep(t.TIME_STEP_S) # sec
        p.setRealTimeSimulation(0)
        p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, 0)   

        m.build()
        obs = m.update()

        return np.array(obs)
        
    def step(self, action):
        self.action = action
        
        self.updateSimulation(action)
        p.stepSimulation()
        obs = m.update()
                
        done = self.compute_done()
        reward = self.compute_reward(action, obs)
        
        self._envStepCounter += 1

        return np.array(obs), reward, done, {}
    
    def decodeAction(self, action):
        raise NotImplementedError
    
    def updateSimulation(self, action): 
        raise NotImplementedError
        
    # Implementations may override
    def compute_reward(self, action, obs):
        velForward = obs[3]
        velSide = obs[4]
        velVert = obs[5]
        
        norm = 0
        for a in action:
            norm += a**2
        norm = norm**0.5
        
        aveLinAmp = obs[33]
        aveAngAmp = obs[34]
        
        return velForward - np.abs(velVert)*0.1 - np.abs(velSide) - norm*.001 - (aveLinAmp + aveAngAmp)*0.01

    def compute_done(self):
        return self._envStepCounter >= 1000 # 10s
        
    def render(self, mode='human', close=False):
        pass


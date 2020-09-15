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

PATH_POLICY = "policy/"
NAME_POLICY = "ppo2_aws_3x_a.zip"

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
        
        obs_min = [-2] * 34
        obs_max = [2] * 34
        self.observation_space = spaces.Box(np.array(obs_min), np.array(obs_max))
                
        self.setRender(render)

    
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

        return np.array(obs[:34])
        
    def step(self, action):        
        self.updateSimulation(action)
        self.last_action = action
        p.stepSimulation()
        obs = m.update()
                
        done = self.compute_done()
        reward = self.compute_reward(action, obs)
        
        self._envStepCounter += 1

        return np.array(obs[:34]), reward, done, {}
    
    def decodeAction(self, action):
        raise NotImplementedError
    
    def updateSimulation(self, action): 
        raise NotImplementedError
        
    # Implementations may override
    def compute_reward(self, action, obs):
        #spin = 0
        #for i in range(3):
        #    spin+= np.abs(obs[i+6])
        #spin = spin / 3
        spin = obs[7] # roll in +X direction
        
        velForward = obs[3] # +X direction
        velSide = obs[4] 
        velVert = obs[5]
        
        diff = 0
        aveAction = 0
        for i in range(len(action)):
            a = action[i]
            aveAction += np.abs(a)
            diff += np.abs(a - self.last_action[i])
        aveAction = aveAction/len(action)
        diff = diff/len(action)
        
        traveled = obs[33]
        
        aveLinAmp = obs[34]
        aveAngAmp = obs[35]
        reach = obs[36]
        
        return traveled + spin*0 + velForward*0 - np.abs(velSide)*0  - np.abs(velVert)*0 - aveAction*0 - (aveLinAmp + aveAngAmp)*0 - diff*0 + reach*0

    def compute_done(self):
        #return self._envStepCounter >= 1000 # 10s
        return self._envStepCounter >= 300
        
    def render(self, mode='human', close=False):
        pass


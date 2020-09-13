import math
import sys
import time

import pybullet as p
import pybullet_data

import model as m
import util as ut

BOUNCE = "bounce"
TIME_STEP_S = 0.01


def configPyBullet():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
    p.resetSimulation()
    p.setGravity(0,0,-9.8) # m/s^2
    p.setTimeStep(TIME_STEP_S) # sec
    p.setRealTimeSimulation(0)
    #planeId = p.loadURDF("plane.urdf")
    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)
    
    return physicsClient#, planeId

def reset():
    p.resetSimulation()
    m.reset()

def step():
    m.reset()
    p.stepSimulation()
    
def main():
    physicsClient = configPyBullet()
    m.build()

def run():
    while (1):
        start = time.time()
        m.impede()
        m.updateSprings()
        m.applySpringForces()
        
        step()
        keys = p.getKeyboardEvents()
        
        stop = time.time()
        delta = stop - start
        if delta < TIME_STEP_S:
            time.sleep(TIME_STEP_S - delta)
      
if __name__ == '__main__':
    arg = None
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg == BOUNCE:
            m.POSITION = (0,0,5)
        
    main()
    run()
    
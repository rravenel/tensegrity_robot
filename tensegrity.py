import math
import time

import pybullet as p
import pybullet_data

import common as cm
import model as m

TIME_STEP_S = 0.01

UIDS = []


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
    
def main():
    physicsClient = configPyBullet()
    
    global UIDS
    UIDS, springs = m.build()

def run():
    while (1):
        start = time.time()
        m.impede(UIDS)
        springs = m.updateSpringPositions(UIDS)
        m.applySpringForces(springs)
        
        p.stepSimulation()
        keys = p.getKeyboardEvents()
        
        stop = time.time()
        delta = stop - start
        if delta < TIME_STEP_S:
            time.sleep(TIME_STEP_S - delta)
      
if __name__ == '__main__':
    main()
    run()
    
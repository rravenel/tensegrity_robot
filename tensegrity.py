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
    
    first = True
    for uid in UIDS:
        if first:
            end0, end1 = cm.strutPose(uid, m.LENGTH_M)
            #cm.createSphere(end0)
        first = not first

def run():
    while (1):
        springs = m.updateSpringPositions(UIDS)
        m.applySpringForces(springs)
        p.stepSimulation()
        keys = p.getKeyboardEvents()
        time.sleep(0.01)
      
if __name__ == '__main__':
    main()
    run()
    
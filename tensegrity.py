import math
import time

import pybullet as p
import pybullet_data

import common as cm

PI = 3.141593
TIME_STEP_S = 0.01
LENGTH_M = 1
RADIUS_M = 0.025
MASS_KG = 0.1
#MASS_KG = 0 # so it doesn't fall

BASE_ID = -1

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

# useful for development
def createSphere(basePosition, baseOrientation=[0, 0, 0]):
    sphereRadius = 0.05
    colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
    
    mass = 0
    visualShapeId = -1
    
    sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId, basePosition,
                                  p.getQuaternionFromEuler(baseOrientation))
    
def createStrut(basePosition, baseOrientation=[0, 0, 0]):
    radius = RADIUS_M
    length = LENGTH_M
    mass = MASS_KG
    
    visualShapeId = -1
    
    colCylinderId = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=length)

    cylinderUid = p.createMultiBody(mass, colCylinderId, visualShapeId, basePosition, baseOrientation)
    
    # TODO for ground contact?
    #p.changeDynamics(cylinderUid,
    #                 -1,
    #                 spinningFriction=0.001,
    #                 rollingFriction=0.001,
    #                 linearDamping=0.0)
                     
    UIDS.append(cylinderUid)
    return cylinderUid

# return global coordinates for ends of strut
def strutPose(uid):
    positionAndOrientation = p.getBasePositionAndOrientation(uid)
    
    position = positionAndOrientation[0]
    orientation = positionAndOrientation[1]
    
    radius = LENGTH_M / 2
    
    # rotate by orientation
    pointA = cm.rotate((0, 0, radius), orientation)
    pointB = cm.rotate((0, 0, -radius), orientation)

    # translate by position
    pointA = cm.translate(pointA, position)
    pointB = cm.translate(pointB, position)

    return pointA, pointB

# push on the end of a strut; end 0 is top end
def push(uid, end, force):
    end0, end1 = strutPose(uid)
    endToUse = end0 if end == 0 else end1
    p.applyExternalForce(uid, BASE_ID, force, endToUse, p.WORLD_FRAME)
    
def report():
    for uid in UIDS:
        positionAndOrientation = p.getBasePositionAndOrientation(uid)
        
        position = positionAndOrientation[0]
        orientation = p.getEulerFromQuaternion(positionAndOrientation[1])
        
        print("\n******************** position: %s" % (position,))
        print("******************** orientation: %s\n" % (orientation,))
        

def main():
    physicsClient = configPyBullet()
    
    basePosition = [0, 0, LENGTH_M/2]
    baseOrientation = [0, 0, 0] # Euler angles [x, y, z]
    strutUid_1 = createStrut(basePosition, p.getQuaternionFromEuler(baseOrientation))
    strutUid_2 = createStrut([0.5, 0, LENGTH_M/2], p.getQuaternionFromEuler(baseOrientation))
    UIDS.append(strutUid_1)
    UIDS.append(strutUid_2)
    
    p1, p2 = strutPose(strutUid_1)
    print(cm.delta(p1, p2))
    
def foo():    
    end1, end2 = strutPose(strutUid_1)
    createSphere(end1)
    createSphere(end2)

    end3, end4 = strutPose(strutUid_2)
    createSphere(end3)
    createSphere(end4)

def run():
    force = (0, 1, 2)
    
    while (1):
        #report()
        #push(UIDS[0], 0, force)
        p.stepSimulation()
        keys = p.getKeyboardEvents()
        time.sleep(0.01)
      
if __name__ == '__main__':
    main()
    run()
    
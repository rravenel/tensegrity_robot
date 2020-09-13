import pybullet as p

import common as cm



X = [1,0,0]
Y = [0,1,0]
Z = [0,0,1]

# N/m
SPRING_K = 25

LENGTH_M = 1
RADIUS_M = 0.025
MASS_KG = 0.1
#MASS_KG = 0 # so it doesn't fall
OFFSET = LENGTH_M * (1/cm.GOLDEN_NUMBER) / 2
POSITION = (0, 0, 10) # global coordinates for center of tensegrity

SPRING_TENSION_N = 5
SPRING_LENGTH_M = 2 * OFFSET - (SPRING_TENSION_N/SPRING_K)
DAMPING = 0.005 # friction/dampening coefficient
IMPEDANCE = 1 - DAMPING

# create position/orientation for pairs of strust for given axis
def placeStruts(axis):
    orientation = p.getQuaternionFromEuler([i * cm.PI/2 for i in axis])
    
    if axis == X:
        axis = Z
    elif axis == Z:
        axis = X    

    center1 = cm.translate([i * OFFSET for i in axis], POSITION)
    center2 = cm.translate([-i * OFFSET for i in axis], POSITION)

    return (center1, orientation), (center2, orientation)

def getCenter(uids):
    sum = [0,0,0]
    for uid in uids:
        positionAndOrientation = p.getBasePositionAndOrientation(uid)
        position = [0]
        sum[0] += position[0]
        sum[1] += position[1]
        sum[2] += position[2]
    return (sum[0] / 6, sum[1] / 6, sum[2] / 6)

def getAverageVelocity(uids):
    sum = [[0,0,0],[0,0,0]]
    for uid in uids:
        linVel, angVel = p.getBaseVelocity(uid)
        sum[0][0] += linVel[0]
        sum[0][1] += linVel[1]
        sum[0][2] += linVel[2]
        sum[1][0] += angVel[0]
        sum[1][1] += angVel[1]
        sum[1][2] += angVel[2]
    return (sum[0][0]/6, sum[0][1]/6, sum[0][2]/6), (sum[1][0]/6, sum[1][1]/6, sum[1][2]/6)

def getStrutEnd(uid, end):
    end0, end1 = cm.strutPose(uid, LENGTH_M)
    if 0 == end:
        return end0
    return end1

def updateSpringPositions(uids):
    springs = [] # list of ((uidA, endA), (uidB, endB))
    springs.append(((uids[0], getStrutEnd(uids[0], 0)),(uids[4], getStrutEnd(uids[4], 0))))
    springs.append(((uids[0], getStrutEnd(uids[0], 0)),(uids[3], getStrutEnd(uids[3], 0))))
    springs.append(((uids[0], getStrutEnd(uids[0], 0)),(uids[3], getStrutEnd(uids[3], 1))))
    springs.append(((uids[0], getStrutEnd(uids[0], 0)),(uids[5], getStrutEnd(uids[5], 0))))
    springs.append(((uids[0], getStrutEnd(uids[0], 1)),(uids[4], getStrutEnd(uids[4], 0))))
    springs.append(((uids[0], getStrutEnd(uids[0], 1)),(uids[2], getStrutEnd(uids[2], 0))))
    springs.append(((uids[0], getStrutEnd(uids[0], 1)),(uids[2], getStrutEnd(uids[2], 1))))
    springs.append(((uids[0], getStrutEnd(uids[0], 1)),(uids[5], getStrutEnd(uids[5], 0))))
    springs.append(((uids[1], getStrutEnd(uids[1], 0)),(uids[4], getStrutEnd(uids[4], 1))))
    springs.append(((uids[1], getStrutEnd(uids[1], 0)),(uids[3], getStrutEnd(uids[3], 0))))
    springs.append(((uids[1], getStrutEnd(uids[1], 0)),(uids[3], getStrutEnd(uids[3], 1))))
    springs.append(((uids[1], getStrutEnd(uids[1], 0)),(uids[5], getStrutEnd(uids[5], 1))))
    springs.append(((uids[1], getStrutEnd(uids[1], 1)),(uids[4], getStrutEnd(uids[4], 1))))
    springs.append(((uids[1], getStrutEnd(uids[1], 1)),(uids[2], getStrutEnd(uids[2], 0))))
    springs.append(((uids[1], getStrutEnd(uids[1], 1)),(uids[2], getStrutEnd(uids[2], 1))))
    springs.append(((uids[1], getStrutEnd(uids[1], 1)),(uids[5], getStrutEnd(uids[5], 1))))
    springs.append(((uids[3], getStrutEnd(uids[3], 0)),(uids[4], getStrutEnd(uids[4], 0))))
    springs.append(((uids[3], getStrutEnd(uids[3], 0)),(uids[4], getStrutEnd(uids[4], 1))))
    springs.append(((uids[3], getStrutEnd(uids[3], 1)),(uids[5], getStrutEnd(uids[5], 0))))
    springs.append(((uids[3], getStrutEnd(uids[3], 1)),(uids[5], getStrutEnd(uids[5], 1))))
    springs.append(((uids[2], getStrutEnd(uids[2], 0)),(uids[4], getStrutEnd(uids[4], 0))))
    springs.append(((uids[2], getStrutEnd(uids[2], 0)),(uids[4], getStrutEnd(uids[4], 1))))
    springs.append(((uids[2], getStrutEnd(uids[2], 1)),(uids[5], getStrutEnd(uids[5], 0))))
    springs.append(((uids[2], getStrutEnd(uids[2], 1)),(uids[5], getStrutEnd(uids[5], 1))))
    
    return springs

def applySpringForces(springs):
    for spring in springs:
        distance = cm.delta(spring[0][1], spring[1][1])
        direction = cm.direction(spring[0][1], spring[1][1])
        force = cm.springForce(SPRING_K, distance - SPRING_LENGTH_M)
        force = (direction[0] * force, direction[1] * force, direction[2] * force)
        cm.push(spring[0][0], spring[0][1], force)
        cm.push(spring[1][0], spring[1][1], [i * -1 for i in force])

def impede(uids):
    for uid in uids:
        linVel, angVel = p.getBaseVelocity(uid)
        
        linVel = [i * IMPEDANCE for i in linVel]
        angVel = [i * IMPEDANCE for i in angVel]

        p.resetBaseVelocity(uid, linVel, angVel)
        

# create an icosahedral tensegrity
def build():
    uids = []
    for strut in placeStruts(X):
        uids.append(cm.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    for strut in placeStruts(Y):
        uids.append(cm.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    for strut in placeStruts(Z):
        uids.append(cm.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    springs = updateSpringPositions(uids)
    
    return uids, springs

import numpy as np
import pybullet as p

import util as ut


X = [1,0,0]
Y = [0,1,0]
Z = [0,0,1]


LENGTH_M = 1
RADIUS_M = 0.025
MASS_KG = 0.1
#MASS_KG = 0 # so it doesn't fall
OFFSET = LENGTH_M * (1/ut.GOLDEN_NUMBER) / 2
POSITION = (0, 0, 0.5) # global coordinates for center of tensegrity

# N/m
SPRING_K = 25
#SPRING_K = 30
SPRING_TENSION_N = 10
SPRING_LENGTH_M = 2 * OFFSET - (SPRING_TENSION_N/SPRING_K)
SPRING_TRAVEL = 0.5
SPRING_LENGTH_MAX_M = (1 + SPRING_TRAVEL) * SPRING_LENGTH_M
SPRING_LENGTH_MIN_M = (1 - SPRING_TRAVEL) * SPRING_LENGTH_M
#DAMPING = 0.005 # friction/dampening coefficient
DAMPING = 0.05 # friction/dampening coefficient
IMPEDANCE = 1 - DAMPING

F_MAX_N = 20 # heuristic impacted by MASS_KG and SPRING_K used for normalization
L_VEL_MAX = 10 # similar to F_MAX_N for linear velocity

UIDS = []
SPRING_LENGTHS = [SPRING_LENGTH_M] * 24
DISTANCES = []
STRUTS = {} 
SPANS = [] # springs are connected across spans

# create position/orientation for pairs of strust for given axis
def placeStruts(axis):
    orientation = p.getQuaternionFromEuler([i * ut.PI/2 for i in axis])
    
    if axis == X:
        axis = Z
    elif axis == Z:
        axis = X    

    center1 = ut.translate([i * OFFSET for i in axis], POSITION)
    center2 = ut.translate([-i * OFFSET for i in axis], POSITION)

    return (center1, orientation), (center2, orientation)

def getCenter():
    total = [0,0,0]
    for uid in UIDS:
        positionAndOrientation = p.getBasePositionAndOrientation(uid)
        position = [0]
        total[0] += position[0]
        total[1] += position[1]
        total[2] += position[2]
    return (total[0] / 6, total[1] / 6, total[2] / 6)

# returns unit vector
def getAveAngPos(positionAndOrientations):
    total = [0,0,0]
    for d in positionAndOrientations:
        orient = d[1]
        orient = ut.rotate((0,0,1), orient) # unit vector quaternion
        total[0] += orient[0]
        total[1] += orient[1]
        total[2] += orient[2]
    pos = (total[0]/6, total[1]/6, total[2]/6)
    norm = (pos[0]**2 + pos[1]**2 + pos[2]**2)**0.5
    return (pos[0]/norm, pos[1]/norm, pos[2]/norm)

def getAverageVelocity(velocities):
    total = [[0,0,0],[0,0,0]]
    amplitude = [[0,0,0],[0,0,0]]
    
    for vel in velocities:
        linVel = vel[0]
        angVel = vel[1]
        
        total[0][0] += linVel[0]
        total[0][1] += linVel[1]
        total[0][2] += linVel[2]
        total[1][0] += angVel[0]
        total[1][1] += angVel[1]
        total[1][2] += angVel[2]
        
        amplitude[0][0] += np.abs(linVel[0])
        amplitude[0][1] += np.abs(linVel[1])
        amplitude[0][2] += np.abs(linVel[2])
        amplitude[1][0] += np.abs(angVel[0])
        amplitude[1][1] += np.abs(angVel[1])
        amplitude[1][2] += np.abs(angVel[2])

    aveLinVel, aveAngVel = (total[0][0]/6, total[0][1]/6, total[0][2]/6), (total[1][0]/6, total[1][1]/6, total[1][2]/6)
    aveLinAmp, aveAngAmp = (amplitude[0][0]/6 + amplitude[0][1]/6 + amplitude[0][2]/6)/3, (amplitude[1][0]/6 + amplitude[1][1]/6 + amplitude[1][2]/6)/3
    
    return aveLinVel, aveAngVel, aveLinAmp, aveAngAmp

def getDistances():
    distances = []
    for span in SPANS:
        distances.append(ut.delta(span[0][1], span[1][1]))
    
    global DISTANCES
    DISTANCES = distances

def getSpans():
    spans = []
        
    spans.append(((UIDS[0], STRUTS[UIDS[0]][0]), (UIDS[4], STRUTS[UIDS[4]][0])))
    spans.append(((UIDS[0], STRUTS[UIDS[0]][0]), (UIDS[3], STRUTS[UIDS[3]][0])))
    spans.append(((UIDS[0], STRUTS[UIDS[0]][0]), (UIDS[3], STRUTS[UIDS[3]][1])))
    spans.append(((UIDS[0], STRUTS[UIDS[0]][0]), (UIDS[5], STRUTS[UIDS[5]][0])))

    spans.append(((UIDS[0], STRUTS[UIDS[0]][1]), (UIDS[4], STRUTS[UIDS[4]][0])))
    spans.append(((UIDS[0], STRUTS[UIDS[0]][1]), (UIDS[2], STRUTS[UIDS[2]][0])))
    spans.append(((UIDS[0], STRUTS[UIDS[0]][1]), (UIDS[2], STRUTS[UIDS[2]][1])))
    spans.append(((UIDS[0], STRUTS[UIDS[0]][1]), (UIDS[5], STRUTS[UIDS[5]][0])))
    
    spans.append(((UIDS[1], STRUTS[UIDS[1]][0]), (UIDS[4], STRUTS[UIDS[4]][1])))
    spans.append(((UIDS[1], STRUTS[UIDS[1]][0]), (UIDS[3], STRUTS[UIDS[3]][0])))
    spans.append(((UIDS[1], STRUTS[UIDS[1]][0]), (UIDS[3], STRUTS[UIDS[3]][1])))
    spans.append(((UIDS[1], STRUTS[UIDS[1]][0]), (UIDS[5], STRUTS[UIDS[5]][1])))
    
    spans.append(((UIDS[1], STRUTS[UIDS[1]][1]), (UIDS[4], STRUTS[UIDS[4]][1])))
    spans.append(((UIDS[1], STRUTS[UIDS[1]][1]), (UIDS[2], STRUTS[UIDS[2]][0])))
    spans.append(((UIDS[1], STRUTS[UIDS[1]][1]), (UIDS[2], STRUTS[UIDS[2]][1])))
    spans.append(((UIDS[1], STRUTS[UIDS[1]][1]), (UIDS[5], STRUTS[UIDS[5]][1])))
    
    spans.append(((UIDS[3], STRUTS[UIDS[3]][0]), (UIDS[4], STRUTS[UIDS[4]][0])))
    spans.append(((UIDS[3], STRUTS[UIDS[3]][0]), (UIDS[4], STRUTS[UIDS[4]][1])))
    spans.append(((UIDS[3], STRUTS[UIDS[3]][1]), (UIDS[5], STRUTS[UIDS[5]][0])))
    spans.append(((UIDS[3], STRUTS[UIDS[3]][1]), (UIDS[5], STRUTS[UIDS[5]][1])))
    
    spans.append(((UIDS[2], STRUTS[UIDS[2]][0]), (UIDS[4], STRUTS[UIDS[4]][0])))
    spans.append(((UIDS[2], STRUTS[UIDS[2]][0]), (UIDS[4], STRUTS[UIDS[4]][1])))
    spans.append(((UIDS[2], STRUTS[UIDS[2]][1]), (UIDS[5], STRUTS[UIDS[5]][0])))
    spans.append(((UIDS[2], STRUTS[UIDS[2]][1]), (UIDS[5], STRUTS[UIDS[5]][1])))
    
    global SPANS
    SPANS = spans

def applyForces(forces):
    for i in range(len(forces)):
        force = forces[i]
        span = SPANS[i]
        direction = ut.direction(span[0][1], span[1][1])
        force = (direction[0] * force, direction[1] * force, direction[2] * force)
        ut.push(span[0][0], span[0][1], force)
        ut.push(span[1][0], span[1][1], [i * -1 for i in force])

def computeForces():
    forces = []
    for i in range(len(DISTANCES)):
        force = ut.springForce(SPRING_K, DISTANCES[i] - SPRING_LENGTHS[i])
        if force < 0:
            force = 0
        forces.append(force)
    return forces

def impede(uid):
    linVel, angVel = p.getBaseVelocity(uid)
    
    linVel = [i * IMPEDANCE for i in linVel]
    angVel = [i * IMPEDANCE for i in angVel]

    p.resetBaseVelocity(uid, linVel, angVel)
    return linVel, angVel

def act(deltas=[]):
    global SPRING_LENGTHS    
    for i in range(len(deltas)):
        d = deltas[i]
        springLength = SPRING_LENGTHS[i]
        springLength += d
        if springLength > SPRING_LENGTH_MAX_M:
            springLength = SPRING_LENGTH_MAX_M
        if springLength < SPRING_LENGTH_MIN_M:
            springLength = SPRING_LENGTH_MIN_M
        SPRING_LENGTHS[i] = springLength
        
    forces = computeForces()
    applyForces(forces)

# input to RL
def update():
    global STRUTS
    positionAndOrientations = []
    velocities = []
    for uid in UIDS:
        velocities.append((impede(uid)))
        positionAndOrientation = p.getBasePositionAndOrientation(uid)
        positionAndOrientations.append(positionAndOrientation)
        STRUTS[uid] = (ut.strutPose(positionAndOrientation, LENGTH_M))

    aveLinVel, aveAngVel, aveLinAmp, aveAngAmp = getAverageVelocity(velocities)
    aveAngPos = getAveAngPos(positionAndOrientations)

    getSpans()
    getDistances()
    forces = computeForces()
        
    observation = []
    for i in range(len(aveAngPos)):
        observation.append(aveAngPos[i])
        
    for i in range(len(aveLinVel)):
        #normalize linear velocity
        observation.append(aveLinVel[i]/L_VEL_MAX)
        
    for i in range(len(aveAngVel)):
        observation.append(aveAngVel[i]/L_VEL_MAX)
        
    for i in range(len(forces)):
        #normalize forces
        observation.append(forces[i]/F_MAX_N)
    
    observation.append(aveLinAmp/L_VEL_MAX)
    observation.append(aveAngAmp/L_VEL_MAX)

    return observation
    

# create an icosahedral tensegrity
def build():
    for strut in placeStruts(X):
        UIDS.append(ut.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    for strut in placeStruts(Y):
        UIDS.append(ut.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    for strut in placeStruts(Z):
        UIDS.append(ut.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
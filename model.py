import pybullet as p

import util as ut



X = [1,0,0]
Y = [0,1,0]
Z = [0,0,1]

# N/m
SPRING_K = 25

LENGTH_M = 1
RADIUS_M = 0.025
MASS_KG = 0.1
#MASS_KG = 0 # so it doesn't fall
OFFSET = LENGTH_M * (1/ut.GOLDEN_NUMBER) / 2
POSITION = (0, 0, 0.5) # global coordinates for center of tensegrity

#SPRING_TENSION_N = 5
SPRING_TENSION_N = 10
SPRING_LENGTH_M = 2 * OFFSET - (SPRING_TENSION_N/SPRING_K)
SPRING_TRAVEL = 0.5
SPRING_LENGTH_MAX_M = 1 + SPRING_TRAVEL * SPRING_LENGTH_M
SPRING_LENGTH_MIM_M = 1 - SPRING_TRAVEL * SPRING_LENGTH_M
DAMPING = 0.005 # friction/dampening coefficient
IMPEDANCE = 1 - DAMPING

UIDS = []
SPRINGS = []
STRUTS = {}

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
    sum = [0,0,0]
    for uid in UIDS:
        positionAndOrientation = p.getBasePositionAndOrientation(uid)
        position = [0]
        sum[0] += position[0]
        sum[1] += position[1]
        sum[2] += position[2]
    return (sum[0] / 6, sum[1] / 6, sum[2] / 6)

def getAverageVelocity():
    sum = [[0,0,0],[0,0,0]]
    for uid in UIDS:
        linVel, angVel = p.getBaseVelocity(uid)
        sum[0][0] += linVel[0]
        sum[0][1] += linVel[1]
        sum[0][2] += linVel[2]
        sum[1][0] += angVel[0]
        sum[1][1] += angVel[1]
        sum[1][2] += angVel[2]
    return (sum[0][0]/6, sum[0][1]/6, sum[0][2]/6), (sum[1][0]/6, sum[1][1]/6, sum[1][2]/6)

def getStrutEnd(uid, end):
    if 0 == len(STRUTS):
        for i in UIDS:
            end0, end1 = ut.strutPose(i, LENGTH_M)
            STRUTS[i] = end0, end1

    strut = STRUTS[uid]
    return strut[end]

def updateSprings(deltas=None):
    global SPRINGS
    if None == deltas:
        lengths=[SPRING_LENGTH_M]*len(UIDS)*4
    else:
        lengths = []
        for i in range(length(SPRINGS)):
            spring = SPRINGS[i]
            length = spring[2] + deltas[i]
            if length > SPRING_LENGTH_MAX_M:
                length = SPRING_LENGTH_MAX_M
            if length < SPRING_LENGTH_MIN_M:
                length = SPRING_LENGTH_MIN_M
        
    springs = [] # list of ((uidA, endA), (uidB, endB))
    springs.append(((UIDS[0], getStrutEnd(UIDS[0], 0)),(UIDS[4], getStrutEnd(UIDS[4], 0)), lengths[0]))
    springs.append(((UIDS[0], getStrutEnd(UIDS[0], 0)),(UIDS[3], getStrutEnd(UIDS[3], 0)), lengths[1]))
    springs.append(((UIDS[0], getStrutEnd(UIDS[0], 0)),(UIDS[3], getStrutEnd(UIDS[3], 1)), lengths[2]))
    springs.append(((UIDS[0], getStrutEnd(UIDS[0], 0)),(UIDS[5], getStrutEnd(UIDS[5], 0)), lengths[3]))
    springs.append(((UIDS[0], getStrutEnd(UIDS[0], 1)),(UIDS[4], getStrutEnd(UIDS[4], 0)), lengths[4]))
    springs.append(((UIDS[0], getStrutEnd(UIDS[0], 1)),(UIDS[2], getStrutEnd(UIDS[2], 0)), lengths[5]))
    springs.append(((UIDS[0], getStrutEnd(UIDS[0], 1)),(UIDS[2], getStrutEnd(UIDS[2], 1)), lengths[6]))
    springs.append(((UIDS[0], getStrutEnd(UIDS[0], 1)),(UIDS[5], getStrutEnd(UIDS[5], 0)), lengths[7]))
    springs.append(((UIDS[1], getStrutEnd(UIDS[1], 0)),(UIDS[4], getStrutEnd(UIDS[4], 1)), lengths[8]))
    springs.append(((UIDS[1], getStrutEnd(UIDS[1], 0)),(UIDS[3], getStrutEnd(UIDS[3], 0)), lengths[9]))
    springs.append(((UIDS[1], getStrutEnd(UIDS[1], 0)),(UIDS[3], getStrutEnd(UIDS[3], 1)), lengths[10]))
    springs.append(((UIDS[1], getStrutEnd(UIDS[1], 0)),(UIDS[5], getStrutEnd(UIDS[5], 1)), lengths[11]))
    springs.append(((UIDS[1], getStrutEnd(UIDS[1], 1)),(UIDS[4], getStrutEnd(UIDS[4], 1)), lengths[12]))
    springs.append(((UIDS[1], getStrutEnd(UIDS[1], 1)),(UIDS[2], getStrutEnd(UIDS[2], 0)), lengths[13]))
    springs.append(((UIDS[1], getStrutEnd(UIDS[1], 1)),(UIDS[2], getStrutEnd(UIDS[2], 1)), lengths[14]))
    springs.append(((UIDS[1], getStrutEnd(UIDS[1], 1)),(UIDS[5], getStrutEnd(UIDS[5], 1)), lengths[15]))
    springs.append(((UIDS[3], getStrutEnd(UIDS[3], 0)),(UIDS[4], getStrutEnd(UIDS[4], 0)), lengths[16]))
    springs.append(((UIDS[3], getStrutEnd(UIDS[3], 0)),(UIDS[4], getStrutEnd(UIDS[4], 1)), lengths[17]))
    springs.append(((UIDS[3], getStrutEnd(UIDS[3], 1)),(UIDS[5], getStrutEnd(UIDS[5], 0)), lengths[18]))
    springs.append(((UIDS[3], getStrutEnd(UIDS[3], 1)),(UIDS[5], getStrutEnd(UIDS[5], 1)), lengths[19]))
    springs.append(((UIDS[2], getStrutEnd(UIDS[2], 0)),(UIDS[4], getStrutEnd(UIDS[4], 0)), lengths[20]))
    springs.append(((UIDS[2], getStrutEnd(UIDS[2], 0)),(UIDS[4], getStrutEnd(UIDS[4], 1)), lengths[21]))
    springs.append(((UIDS[2], getStrutEnd(UIDS[2], 1)),(UIDS[5], getStrutEnd(UIDS[5], 0)), lengths[22]))
    springs.append(((UIDS[2], getStrutEnd(UIDS[2], 1)),(UIDS[5], getStrutEnd(UIDS[5], 1)), lengths[23]))
    
    SPRINGS = springs

def applySpringForces():
    for spring in SPRINGS:
        distance = ut.delta(spring[0][1], spring[1][1])
        direction = ut.direction(spring[0][1], spring[1][1])
        force = ut.springForce(SPRING_K, distance - SPRING_LENGTH_M)
        force = (direction[0] * force, direction[1] * force, direction[2] * force)
        ut.push(spring[0][0], spring[0][1], force)
        ut.push(spring[1][0], spring[1][1], [i * -1 for i in force])

def impede():
    for uid in UIDS:
        linVel, angVel = p.getBaseVelocity(uid)
        
        linVel = [i * IMPEDANCE for i in linVel]
        angVel = [i * IMPEDANCE for i in angVel]

        p.resetBaseVelocity(uid, linVel, angVel)
        

# create an icosahedral tensegrity
def build():
    for strut in placeStruts(X):
        UIDS.append(ut.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    for strut in placeStruts(Y):
        UIDS.append(ut.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    for strut in placeStruts(Z):
        UIDS.append(ut.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    global SPRINGS
    SPRINGS = updateSprings()

def reset():
    STRUTS.clear()
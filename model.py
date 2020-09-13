import pybullet as p

import common as cm



X = [1,0,0]
Y = [0,1,0]
Z = [0,0,1]

LENGTH_M = 1
RADIUS_M = 0.025
#MASS_KG = 0.1
MASS_KG = 0 # so it doesn't fall

OFFSET = LENGTH_M * (1/cm.GOLDEN_NUMBER) / 2
POSITION = (0, 0, 0.5) # global coordinates for center of tensegrity

# create position/orientation for pairs of strust for given axis
def locateStruts(axis):
    orientation = p.getQuaternionFromEuler([i * cm.PI/2 for i in axis])
    
    if axis == X:
        axis = Z
    elif axis == Z:
        axis = X    

    center1 = cm.translate([i * OFFSET for i in axis], POSITION)
    center2 = cm.translate([-i * OFFSET for i in axis], POSITION)

    return (center1, orientation), (center2, orientation)

# create an icosahedral tensegrity
def build():
    UIDS = []
    for strut in locateStruts(X):
        UIDS.append(cm.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    for strut in locateStruts(Y):
        UIDS.append(cm.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
    
    for strut in locateStruts(Z):
        UIDS.append(cm.createStrut(RADIUS_M, LENGTH_M, MASS_KG, strut[0], strut[1]))
        
    return UIDS
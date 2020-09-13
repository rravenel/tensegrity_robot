import pybullet as p

PI = 3.141593
GOLDEN_NUMBER = 1.618 

BASE_ID = -1

def rad2Deg(rad):
    return rad/PI * 180

def hamiltonProduct(q1, q2):
    w = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]
    i = q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1]
    j = q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0]
    k = q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3]
    return (i, j, k, w)

# PyBullet represents quaternions: [x, y, z, w]
def conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])

def rotate(position, orientation):
    #convert position to vector quaternion
    if 3 == len(position):
        position = (position[0], position[1], position[2], 0)
    return hamiltonProduct(hamiltonProduct(orientation, position), conjugate(orientation))

def translate(p1, p2):
    return (p1[0] + p2[0], p1[1] + p2[1], p1[2] + p2[2])

def delta(p1, p2):
    return (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])


# push on the end of a strut; end 0 is top end
def push(uid, end, force):
    end0, end1 = strutPose(uid)
    endToUse = end0 if end == 0 else end1
    p.applyExternalForce(uid, BASE_ID, force, endToUse, p.WORLD_FRAME)

# useful for development
def createSphere(basePosition, baseOrientation=[0, 0, 0]):
    sphereRadius = 0.05
    colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
    
    mass = 0
    visualShapeId = -1
    
    sphereUid = p.createMultiBody(mass, colSphereId, visualShapeId, basePosition,
                                  p.getQuaternionFromEuler(baseOrientation))
    
def createStrut(radius, length, mass, basePosition, baseOrientation=[0, 0, 0, 1]):    
    visualShapeId = -1
    
    colCylinderId = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=length)

    cylinderUid = p.createMultiBody(mass, colCylinderId, visualShapeId, basePosition, baseOrientation)
    
    # TODO for ground contact?
    #p.changeDynamics(cylinderUid,
    #                 -1,
    #                 spinningFriction=0.001,
    #                 rollingFriction=0.001,
    #                 linearDamping=0.0)
                     
    return cylinderUid

# return global coordinates for ends of strut
def strutPose(uid, length): # not sure how to get link geometry
    positionAndOrientation = p.getBasePositionAndOrientation(uid)
    
    position = positionAndOrientation[0]
    orientation = positionAndOrientation[1]
    
    radius = length / 2
    
    # rotate by orientation
    pointA = cm.rotate((0, 0, radius), orientation)
    pointB = cm.rotate((0, 0, -radius), orientation)

    # translate by position
    pointA = cm.translate(pointA, position)
    pointB = cm.translate(pointB, position)

    return pointA, pointB



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
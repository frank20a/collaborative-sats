import casadi as cs


def quaternion_multiply(quaternion1, quaternion0):
    res = cs.SX(4, 1)
    
    res[3] = quaternion1[3]*quaternion0[3] - quaternion1[0]*quaternion0[0] - quaternion1[1]*quaternion0[1] - quaternion1[2]*quaternion0[2]
    res[0] = quaternion1[3]*quaternion0[0] + quaternion1[0]*quaternion0[3] + quaternion1[1]*quaternion0[2] - quaternion1[2]*quaternion0[1]
    res[1] = quaternion1[3]*quaternion0[1] - quaternion1[0]*quaternion0[2] + quaternion1[1]*quaternion0[3] + quaternion1[2]*quaternion0[0]
    res[2] = quaternion1[3]*quaternion0[2] + quaternion1[0]*quaternion0[1] - quaternion1[1]*quaternion0[0] + quaternion1[2]*quaternion0[3]
    
    return res


def quaternion_inverse(quaternion):
    return quaternion_conjugate(quaternion) / quaternion_norm(quaternion)


def quaternion_conjugate(quaternion):
    res = cs.SX(4, 1)
    res[0] = -quaternion[0]
    res[1] = -quaternion[1]
    res[2] = -quaternion[2]
    res[3] = quaternion[3]
    
    return res
    

def quaternion_norm(quaternion):
    return cs.sqrt(quaternion[0]**2 + quaternion[1]**2 + quaternion[2]**2 + quaternion[3]**2)


def euler_from_quaternion(quaternion):
    res = cs.SX(3, 1)

    res[0] = cs.atan2(
        2 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2]),
        quaternion[3]**2 - quaternion[0]**2 - quaternion[1]**2 + quaternion[2]**2
    )
    res[1] = cs.asin(2 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]))
    res[2] = cs.atan2(
        2 * (quaternion[3] * quaternion[2] - quaternion[0] * quaternion[1]),
        quaternion[3]**2 + quaternion[0]**2 - quaternion[1]**2 - quaternion[2]**2
    )

    return res
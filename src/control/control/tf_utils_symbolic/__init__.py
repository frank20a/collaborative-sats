from .quaternions import *
import casadi as cs


def vector_rotate_quaternion(v, q):
    res = cs.SX(4, 1)
    res[0:3] = v
    
    res = quaternion_multiply(
        q,
        res
    )
    
    res = quaternion_multiply(
        res,
        quaternion_inverse(q)
    )
    
    return res[0:3]

from .quaternions import *
import casadi as cs


def vector_rotate_quaternion(v, q):
    res = cs.SX(3)
    
    res = quaternion_multiply(
        (q[0], q[1], q[2], q[3]),
        (v[0], v[1], v[2], 0)
    )
    
    res = quaternion_multiply(
        res,
        quaternion_inverse((q[0], q[1], q[2], q[3]))
    )
    
    return res[0:3]

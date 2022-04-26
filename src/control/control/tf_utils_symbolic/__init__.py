from .quaternions import *
import casadi as cs
import numpy as np


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


def matrix_from_euler(eul):
    Rz = cs.SX(3, 3)
    Rz[0, 0] = cs.cos(eul[2])
    Rz[0, 1] = -cs.sin(eul[2])
    Rz[1, 0] = cs.sin(eul[2])
    Rz[1, 1] =  cs.cos(eul[2])
    Rz[2, 2] = 1.0
    
    Ry = cs.SX(3, 3)
    Ry[0, 0] = cs.cos(eul[1])
    Ry[0, 2] = cs.sin(eul[1])
    Ry[1, 1] = 1.0
    Ry[2, 0] = -cs.sin(eul[1])
    Ry[2, 2] = cs.cos(eul[1])
    
    Rx = cs.SX(3, 3)
    Rx[0, 0] = 1.0
    Rx[1, 1] = cs.cos(eul[0])
    Rx[1, 2] = -cs.sin(eul[0])
    Rx[2, 1] = cs.sin(eul[0])
    Rx[2, 2] = cs.cos(eul[0])
    
    return Rz

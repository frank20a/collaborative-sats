# Ported from tf_transformation for use with casadi symbolic variables
# https://github.com/DLu/tf_transformations

import casadi as cs
import numpy
from transforms3d.quaternions import qmult


def quaternion_multiply(quaternion1, quaternion0):
    """
    Return multiplication of two quaternions.

    >>> q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    >>> numpy.allclose(q, [-44, -14, 48, 28])
    True

    """
    x2, y2, z2, w2 = quaternion0
    x1, y1, z1, w1 = quaternion1
    
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2
    
    return cs.SX([x, y, z, w])


def quaternion_inverse(quaternion):
    """
    Return inverse of quaternion.

    >>> q0 = random_quaternion()
    >>> q1 = quaternion_inverse(q0)
    >>> numpy.allclose(quaternion_multiply(q0, q1), [0, 0, 0, 1])
    True

    """
    return quaternion_conjugate(quaternion) / cs.dot(quaternion, quaternion)


def quaternion_conjugate(quaternion):
    """
    Return conjugate of quaternion.

    >>> q0 = random_quaternion()
    >>> q1 = quaternion_conjugate(q0)
    >>> q1[3] == q0[3] and all(q1[:3] == -q0[:3])
    True

    """
    return cs.SX([
        -quaternion[0], 
        -quaternion[1],
        -quaternion[2], 
        quaternion[3]
    ])
    

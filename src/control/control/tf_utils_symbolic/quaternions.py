# Ported from tf_transformation for use with casadi symbolic variables
# https://github.com/DLu/tf_transformations

import casadi as cs
from numpy import triu as np_triu, ones as np_ones

# Ported from transforms3d
# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

_EPS4 = 1e-13

_shearers = {}
for n in range(1,11):
    x = (n**2 + n)/2.0
    i = n+1
    _shearers[x] = (i, np_triu(np_ones((i,i)), 1).astype(bool))
    

TRANSLATION_IDENTITY = [0.0, 0.0, 0.0]
ZOOM_IDENTITY = [1.0, 1.0, 1.0]


def quaternion_multiply(quaternion1, quaternion0):
    """
    Return multiplication of two quaternions.

    >>> q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    >>> numpy.allclose(q, [-44, -14, 48, 28])
    True

    """
    
    res = cs.SX(4, 1)
    
    res[3] = quaternion1[3]*quaternion0[3] - quaternion1[0]*quaternion0[0] - quaternion1[1]*quaternion0[1] - quaternion1[2]*quaternion0[2]
    res[0] = quaternion1[3]*quaternion0[0] + quaternion1[0]*quaternion0[3] + quaternion1[1]*quaternion0[2] - quaternion1[2]*quaternion0[1]
    res[1] = quaternion1[3]*quaternion0[1] + quaternion1[1]*quaternion0[3] + quaternion1[2]*quaternion0[0] - quaternion1[0]*quaternion0[2]
    res[2] = quaternion1[3]*quaternion0[2] + quaternion1[2]*quaternion0[3] + quaternion1[0]*quaternion0[1] - quaternion1[1]*quaternion0[0]
    
    return res


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
    
    res = cs.SX(4, 1)
    res[0] = -quaternion[0]
    res[1] = -quaternion[1]
    res[2] = -quaternion[2]
    res[3] = quaternion[3]
    
    return res
    
    
def euler_from_quaternion(quaternion, axes='sxyz'):
    """
    Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)


def euler_from_matrix(matrix, axes='sxyz'):
    """
    Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    return mat2euler(matrix, axes=axes)


def quaternion_matrix(quaternion):
    """
    Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    
    rot = cs.SX(4, 1)
    rot[0] = quaternion[3]
    rot[1] = quaternion[0]
    rot[2] = quaternion[1]
    rot[3] = quaternion[2]
    
    rotation_matrix = quat2mat(rot)
    return compose(
        TRANSLATION_IDENTITY,
        rotation_matrix,
        ZOOM_IDENTITY
    )


# Ported from transforms3d
def mat2euler(mat, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    Note that many Euler angle triplets can describe one matrix.

    Parameters
    ----------
    mat : array-like shape (3, 3) or (4, 4)
        Rotation matrix or affine.
    axes : str, optional
        Axis specification; one of 24 axis sequences as string or encoded
        tuple - e.g. ``sxyz`` (the default).

    Returns
    -------
    ai : float
        First rotation angle (according to `axes`).
    aj : float
        Second rotation angle (according to `axes`).
    ak : float
        Third rotation angle (according to `axes`).

    Examples
    --------
    >>> R0 = euler2mat(1, 2, 3, 'syxz')
    >>> al, be, ga = mat2euler(R0, 'syxz')
    >>> R1 = euler2mat(al, be, ga, 'syxz')
    >>> np.allclose(R0, R1)
    True
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = cs.SX(mat)[:3, :3]
    if repetition:
        sy = cs.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS4:
            ax = cs.atan2( M[i, j],  M[i, k])
            ay = cs.atan2( sy,       M[i, i])
            az = cs.atan2( M[j, i], -M[k, i])
        else:
            ax = cs.atan2(-M[j, k],  M[j, j])
            ay = cs.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = cs.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        ax = cs.atan2( M[k, j],  M[k, k])
        ay = cs.atan2(-M[k, i],  cy)
        az = cs.atan2( M[j, i],  M[i, i])

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


# Ported from transforms3d
def quat2mat(q):
    ''' Calculate rotation matrix corresponding to quaternion

    Parameters
    ----------
    q : 4 element array-like

    Returns
    -------
    M : (3,3) array
      Rotation matrix corresponding to input quaternion *q*

    Notes
    -----
    Rotation matrix applies to column vectors, and is applied to the
    left of coordinate vectors.  The algorithm here allows quaternions that
    have not been normalized.

    References
    ----------
    Algorithm from http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion

    Examples
    --------
    >>> import numpy as np
    >>> M = quat2mat([1, 0, 0, 0]) # Identity quaternion
    >>> np.allclose(M, np.eye(3))
    True
    >>> M = quat2mat([0, 1, 0, 0]) # 180 degree rotn around axis 0
    >>> np.allclose(M, np.diag([1, -1, -1]))
    True
    '''
    # w, x, y, z = q
    Nq = q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2
    
    s = 2.0/Nq
    X = q[1]*s
    Y = q[2]*s
    Z = q[3]*s
    wX = q[0]*X; wY = q[0]*Y; wZ = q[0]*Z
    xX = q[1]*X; xY = q[1]*Y; xZ = q[1]*Z
    yY = q[2]*Y; yZ = q[2]*Z; zZ = q[3]*Z
    
    res = cs.SX(3, 3)
    res[0, 0] = 1.0 - (yY + zZ)
    res[0, 1] = xY - wZ
    res[0, 2] = xZ + wY
    res[1, 0] = xY + wZ
    res[1, 1] = 1.0 - (xX + zZ)
    res[1, 2] = yZ - wX
    res[2, 0] = xZ - wY
    res[2, 1] = yZ + wX
    res[2, 2] = 1.0 - (xX + yY)
    return res


# Ported from transforms3d
def compose(T, R, Z, S=None):
    ''' Compose translations, rotations, zooms, [shears]  to affine

    Parameters
    ----------
    T : array-like shape (N,)
        Translations, where N is usually 3 (3D case)
    R : array-like shape (N,N)
        Rotation matrix where N is usually 3 (3D case)
    Z : array-like shape (N,)
        Zooms, where N is usually 3 (3D case)
    S : array-like, shape (P,), optional
       Shear vector, such that shears fill upper triangle above
       diagonal to form shear matrix.  P is the (N-2)th Triangular
       number, which happens to be 3 for a 4x4 affine (3D case)

    Returns
    -------
    A : array, shape (N+1, N+1)
        Affine transformation matrix where N usually == 3
        (3D case)

    Examples
    --------
    >>> T = [20, 30, 40] # translations
    >>> R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]] # rotation matrix
    >>> Z = [2.0, 3.0, 4.0] # zooms
    >>> A = compose(T, R, Z)
    >>> A
    array([[  0.,  -3.,   0.,  20.],
           [  2.,   0.,   0.,  30.],
           [  0.,   0.,   4.,  40.],
           [  0.,   0.,   0.,   1.]])
    >>> S = np.zeros(3)
    >>> B = compose(T, R, Z, S)
    >>> np.all(A == B)
    True

    A null set

    >>> compose(np.zeros(3), np.eye(3), np.ones(3), np.zeros(3))
    array([[ 1.,  0.,  0.,  0.],
           [ 0.,  1.,  0.,  0.],
           [ 0.,  0.,  1.,  0.],
           [ 0.,  0.,  0.,  1.]])
    '''
    n = len(T)
    R = cs.SX(R)
    if R.shape != (n,n):
        raise ValueError('Expecting shape (%d,%d) for rotations' % (n,n))
    A = cs.SX.eye(n+1)
    if not S is None:
        Smat = striu2mat(S)
        ZS = cs.dot(cs.diag(cs.SX(Z)), Smat)
    else:
        ZS = cs.diag(Z)
    A[:n,:n] = cs.dot(R, ZS)
    A[:n,n] = T[:]
    return A


# Ported from transforms3d
def striu2mat(striu):
    ''' Construct shear matrix from upper triangular vector

    Parameters
    ----------
    striu : array, shape (N,)
       vector giving triangle above diagonal of shear matrix.

    Returns
    -------
    SM : array, shape (N, N)
       shear matrix

    Examples
    --------
    >>> S = [0.1, 0.2, 0.3]
    >>> striu2mat(S)
    array([[ 1. ,  0.1,  0.2],
           [ 0. ,  1. ,  0.3],
           [ 0. ,  0. ,  1. ]])
    >>> striu2mat([1])
    array([[ 1.,  1.],
           [ 0.,  1.]])
    >>> striu2mat([1, 2])
    Traceback (most recent call last):
       ...
    ValueError: 2 is a strange number of shear elements

    Notes
    -----
    Shear lengths are triangular numbers.

    See http://en.wikipedia.org/wiki/Triangular_number
    '''
    n = len(striu)
    # cached case
    if n in _shearers:
        N, inds = _shearers[n]
    else: # General case
        N = ((-1+cs.sqrt(8*n+1))/2.0)+1 # n+1 th root
        if N != cs.floor(N):
            raise ValueError('%d is a strange number of shear elements' %
                             n)
        N = int(N)
        inds = np_triu(np_ones((N,N)), 1).astype(bool)
    M = cs.SX.eye(N)
    M[inds] = striu
    return M
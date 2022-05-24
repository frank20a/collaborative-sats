import numpy as np
import casadi as cs

from slider_experiment.parameters import force

nt = 8              # Number of thrusters
ni = 3              # Number of inputs
A_ = np.array([     # Thruster matrix
    [-1,     0,     1,     0,     1,     0,    -1,     0   ],
    [ 0,     1,     0,     1,     0,    -1,     0,    -1   ],
    [-0.14,  0.14,  0.14, -0.14, -0.14,  0.14,  0.14, -0.14],
], dtype=np.float64)
A = cs.SX(A_)
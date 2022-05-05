import numpy as np
from numpy import NaN
import casadi as cs

force = 2.0
torque = 5e-2
dt = 1.0/10
m = 30.0

Icm = cs.SX(3, 3)
for i in range(3): 
    Icm[i, i] = [m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.25**2) / 12][i]

mpc_horizon = 30
nu = 6
nx = 13
mpc_state_weights = [
    30,     # position
    1,      # velocity
    65,     # orientation (exponential cost)
    5       # omega
]
mpc_input_weights = [   
    1,      # force
    40      # torque
]
mpc_final_weights = [50 * i for i in mpc_state_weights]
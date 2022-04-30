import numpy as np
from numpy import NaN
import casadi as cs

force = 2.0
torque = 5e-2
dt = 1.0/5
m = 30.0

Icm = cs.SX(3, 3)
for i in range(3): 
    Icm[i, i] = [30 * (0.25**2 + 0.05**2) / 12, 30 * (0.25**2 + 0.05**2) / 12, 30 * (0.25**2 + 0.25**2) / 12][i]

mpc_horizon = 30
nu = 6
nx = 13
# mpc_cost_weights = [1e14, 1e14, 1e14, 5e8, 5e8, 5e8]
mpc_state_weights = [5, 5, 5, 1e-4, 1e-4, 1e-4, 2, 2, 2, NaN, 5e-1, 5e-1, 5e-1]
mpc_input_weights = [30, 30, 30, 2e4, 2e3, 2e4]
mpc_final_weights = [1e2 * i for i in mpc_state_weights]
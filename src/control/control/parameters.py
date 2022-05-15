import numpy as np
from numpy import NaN
import casadi as cs

force = 2.0
torque = 5e-2
m = 30.0

Icm = cs.SX(3, 3)
for i in range(3): 
    Icm[i, i] = [m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.25**2) / 12][i]

nu = 6              # Number of control inputs
nx = 13             # Number of states

solo_tuning = {
    'mpc_state_weights': [
        225,     # position
        0,      # velocity
        105,     # orientation
        0       # omega
    ],
    'mpc_input_weights': [   
        1.5,    # force
        40      # torque
    ],
}
solo_tuning['mpc_final_weights'] = [50 * i for i in solo_tuning['mpc_state_weights']]

dueto_tuning = {
    'mpc_state_weights': [
        350,     # position
        0,      # velocity
        175,     # orientation
        0       # omega
    ],
    'mpc_input_weights': [   
        2,      # force
        50      # torque
    ],
}
dueto_tuning['mpc_final_weights'] = [20 * i for i in solo_tuning['mpc_state_weights']]
import numpy as np
from numpy import NaN
import casadi as cs

force = 1.4
torque = 0.56
m = 30.0

Icm = cs.SX(3, 3)
for i in range(3): 
    Icm[i, i] = [m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.25**2) / 12][i]

nu = 6              # Number of control inputs
nx = 13             # Number of states

solo_tuning = {
    'mpc_state_weights': [
        65,    # position
        0,      # velocity
        35,    # orientation
        0       # omega
    ],
    'mpc_input_weights': [   
        3.5,    # force
        40      # torque
    ],
}
solo_tuning['mpc_final_weights'] = [50 * i for i in solo_tuning['mpc_state_weights']]


dueto_tuning = {
    'mpc_state_weights': [
        65,    # position
        0,      # velocity
        45,    # orientation
        0       # omega
    ],
    'mpc_input_weights': [   
        3.5,      # force
        40      # torque
    ],
}
dueto_tuning['mpc_final_weights'] = [50 * i for i in solo_tuning['mpc_state_weights']]


slider_tuning = {
    'mpc_state_weights': [
        70,    # position
        0,      # velocity
        7,    # orientation
        0       # omega
    ],
    'mpc_input_weights': [   
        50,      # force
        150      # torque
    ],
}
slider_tuning['mpc_final_weights'] = [50 * i for i in solo_tuning['mpc_state_weights']]
import numpy as np
from numpy import NaN
import casadi as cs

force = 2.0
torque = 5e-2
m = 30.0

mpc_state_weights = [
    55,     # position
    0,      # velocity
    70,     # orientation
    0       # omega
]
mpc_input_weights = [   
    3,    # force
    50      # torque
]
mpc_final_weights = [50 * i for i in mpc_state_weights]
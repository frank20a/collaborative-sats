import numpy as np
from numpy import NaN
import casadi as cs

force = 2.0
torque = 5e-2
dt = 1.0/10
m = 30.0

mpc_state_weights = [
    30,     # position
    0,      # velocity
    65,     # orientation (exponential cost)
    0       # omega
]
mpc_input_weights = [   
    1,      # force
    40      # torque
]
mpc_final_weights = [50 * i for i in mpc_state_weights]
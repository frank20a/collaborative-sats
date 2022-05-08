import casadi as cs
from control.parameters import force, torque, dt

m = 30.0

Icm = cs.SX(3, 3)
for i in range(3): 
    Icm[i, i] = [m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.25**2) / 12][i]

nc = 1              # Number of chasers
mpc_horizon = 30    # Number of MPC steps
nu = 6              # Number of control inputs
nx = 13             # Number of states
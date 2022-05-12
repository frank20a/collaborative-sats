import casadi as cs
from control.parameters import force, torque

m = 30.0
dt = 1.0/5

Icm = cs.SX(3, 3)
for i in range(3): 
    Icm[i, i] = [m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.05**2) / 12, m * (0.25**2 + 0.25**2) / 12][i]

mpc_horizon = 30
nu = 6
nx = 13